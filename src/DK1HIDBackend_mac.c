#include "DK1HIDBackend.h"
#include "DK1Tracker/DK1Error.h"

#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/hid/IOHIDDevice.h>
#include <IOKit/hid/IOHIDKeys.h>
#include <CoreFoundation/CoreFoundation.h>

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Per the DK1 tracker firmware specification, input Report ID 1 is 62 bytes.
// We pre-allocate a 64-byte input report buffer so the IOHID callback can
// write into it without ever doing a heap allocation on the HID thread.
#define DK1_INPUT_REPORT_BUF_LEN 64

typedef struct {
    IOHIDManagerRef manager;
    IOHIDDeviceRef  device;       // retained while open
    bool            device_opened;

    pthread_t       loop_thread;
    bool            loop_thread_running;
    pthread_mutex_t ready_mutex;
    pthread_cond_t  ready_cond;
    bool            run_loop_ready;
    CFRunLoopRef    run_loop;     // retained while thread is running

    void          (*report_cb)(const uint8_t *data, size_t length, void *user_data);
    void           *user_data;

    // Buffer supplied to IOHID for incoming reports. The device may
    // provide the Report ID as a separate argument; in that case the
    // callback must construct a normalized report. This buffer is
    // used as the raw data sink.
    uint8_t        *input_report_buf;
    // Temporary buffer for constructing a normalized 62‑byte report.
    uint8_t         normalized_report_buf[DK1_INPUT_REPORT_BUF_LEN];
} MacHIDImpl;

// Forward declarations.
static void hid_input_report_callback(
    void *context,
    IOReturn result,
    void *sender,
    IOHIDReportType type,
    uint32_t reportID,
    uint8_t *report,
    CFIndex reportLength
);

static int mac_open(DK1HIDBackend *backend, uint16_t vid, uint16_t pid) {
    MacHIDImpl *impl = calloc(1, sizeof(MacHIDImpl));
    if (!impl) return DK1_ERROR_IO;

    if (pthread_mutex_init(&impl->ready_mutex, NULL) != 0) {
        free(impl);
        return DK1_ERROR_IO;
    }
    if (pthread_cond_init(&impl->ready_cond, NULL) != 0) {
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    impl->input_report_buf = malloc(DK1_INPUT_REPORT_BUF_LEN);
    if (!impl->input_report_buf) {
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    impl->manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!impl->manager) {
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    CFMutableDictionaryRef match = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks
    );
    if (!match) {
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    int v_id = (int)vid;
    int p_id = (int)pid;
    CFNumberRef v_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &v_id);
    CFNumberRef p_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &p_id);
    if (!v_num || !p_num) {
        if (v_num) CFRelease(v_num);
        if (p_num) CFRelease(p_num);
        CFRelease(match);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey),  v_num);
    CFDictionarySetValue(match, CFSTR(kIOHIDProductIDKey), p_num);
    IOHIDManagerSetDeviceMatching(impl->manager, match);

    CFRelease(v_num);
    CFRelease(p_num);
    CFRelease(match);

    IOReturn rc = IOHIDManagerOpen(impl->manager, kIOHIDOptionsTypeNone);
    if (rc != kIOReturnSuccess) {
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_OPEN_FAILED;
    }

    // Pull matching devices out of the manager. CFSet is unordered, so we
    // cannot use CFSetGetValue(set, index); we must materialize the values
    // into a C array first, then retain the chosen device.
    CFSetRef device_set = IOHIDManagerCopyDevices(impl->manager);
    if (!device_set) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }

    CFIndex count = CFSetGetCount(device_set);
    fprintf(stderr, "[DEBUG] Found %lld device(s) matching VID/PID\n", (long long)count);
    if (count > 0) {
        CFTypeRef *items = malloc(count * sizeof(CFTypeRef));
        CFSetGetValues(device_set, items);
        for (CFIndex i = 0; i < count; ++i) {
            IOHIDDeviceRef dev = (IOHIDDeviceRef)items[i];
            CFNumberRef v = IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDVendorIDKey));
            CFNumberRef p = IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDProductIDKey));
            int vid=0, pid=0;
            if (v) CFNumberGetValue(v, kCFNumberIntType, &vid);
            if (p) CFNumberGetValue(p, kCFNumberIntType, &pid);
            fprintf(stderr, "  device %lld: VID=%04X PID=%04X\n", (long long)i, vid, pid);
        }
        free(items);
    }
    if (count <= 0) {
        CFRelease(device_set);
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }

    IOHIDDeviceRef *devices = calloc((size_t)count, sizeof(IOHIDDeviceRef));
    if (!devices) {
        CFRelease(device_set);
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_IO;
    }

    CFSetGetValues(device_set, (const void **)devices);
    impl->device = devices[0];        // borrow; retain below
    free(devices);
    CFRelease(device_set);

    if (!impl->device) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }

    CFRetain(impl->device);

    // Open the device so we can exchange feature and input reports.
    rc = IOHIDDeviceOpen(impl->device, kIOHIDOptionsTypeNone);
    if (rc != kIOReturnSuccess) {
        fprintf(stderr, "[DEBUG] IOHIDDeviceOpen failed: 0x%08X\n", (unsigned)rc);
        CFRelease(impl->device);
        impl->device = NULL;
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        pthread_cond_destroy(&impl->ready_cond);
        pthread_mutex_destroy(&impl->ready_mutex);
        free(impl);
        return DK1_ERROR_OPEN_FAILED;
    }
    fprintf(stderr, "[DEBUG] IOHIDDeviceOpen succeeded, device opened.\n");
    impl->device_opened = true;

    backend->impl = impl;
    return DK1_OK;
}

static void mac_close(DK1HIDBackend *backend) {
    if (!backend || !backend->impl) return;
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;

    // If the run loop thread is still alive, stop and join it first so we
    // are not tearing down state it may be touching.
    if (impl->loop_thread_running) {
        if (impl->run_loop) {
            CFRunLoopStop(impl->run_loop);
        }
        pthread_join(impl->loop_thread, NULL);
        impl->loop_thread_running = false;
    }
    if (impl->run_loop) {
        CFRelease(impl->run_loop);
        impl->run_loop = NULL;
    }

    if (impl->device) {
        if (impl->device_opened) {
            IOHIDDeviceClose(impl->device, kIOHIDOptionsTypeNone);
            impl->device_opened = false;
        }
        CFRelease(impl->device);
        impl->device = NULL;
    }

    if (impl->manager) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        impl->manager = NULL;
    }

    if (impl->input_report_buf) {
        free(impl->input_report_buf);
        impl->input_report_buf = NULL;
    }

    pthread_cond_destroy(&impl->ready_cond);
    pthread_mutex_destroy(&impl->ready_mutex);

    free(impl);
    backend->impl = NULL;
}

static int mac_is_open(const DK1HIDBackend *backend) {
    return (backend && backend->impl) ? DK1_OK : DK1_ERROR_NOT_OPEN;
}

static void *run_loop_thread_main(void *arg) {
    MacHIDImpl *impl = (MacHIDImpl *)arg;
    if (!impl) return NULL;

    // Capture the run loop this thread owns so stop() can wake us up.
    CFRunLoopRef rl = CFRunLoopGetCurrent();
    CFRetain(rl);

    pthread_mutex_lock(&impl->ready_mutex);
    impl->run_loop = rl;
    impl->run_loop_ready = true;
    pthread_cond_signal(&impl->ready_cond);
    pthread_mutex_unlock(&impl->ready_mutex);

    CFRunLoopRun();

    // CFRunLoopStop has returned control here. Unschedule and unregister
    // before we hand control back to the joiner.
    if (impl->device) {
        IOHIDDeviceUnscheduleFromRunLoop(impl->device, rl, kCFRunLoopDefaultMode);
        IOHIDDeviceRegisterInputReportCallback(
            impl->device, impl->input_report_buf, DK1_INPUT_REPORT_BUF_LEN,
            NULL, impl
        );
    }

    // Drop our own reference; close() will release the impl's reference.
    CFRelease(rl);
    return NULL;
}

static void hid_input_report_callback_normalized(
      void *context, IOReturn result, void *sender, IOHIDReportType type,
      uint32_t reportID, uint8_t *report, CFIndex reportLength)
{
    (void)result; (void)sender; (void)type;
    MacHIDImpl *impl = (MacHIDImpl *)context;
    if (!impl || !impl->report_cb) return;

    const uint8_t *data = report;
    size_t data_len = (size_t)reportLength;
    if (reportLength == 61 && reportID == 1) {
        impl->input_report_buf[0] = 1;
        memcpy(impl->input_report_buf + 1, report, 61);
        data = impl->input_report_buf;
        data_len = 62;
    } else if (reportLength >= 62 && report[0] == 1) {
        // already normalized
    } else {
        // forward as-is
    }
    impl->report_cb(data, data_len, impl->user_data);
}

static int mac_start(DK1HIDBackend *backend) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device || !impl->device_opened) {
        return DK1_ERROR_NOT_OPEN;
    }
    if (impl->loop_thread_running) {
        return DK1_OK; // already started
    }

    impl->run_loop_ready = false;

    int rc = pthread_create(&impl->loop_thread, NULL, run_loop_thread_main, impl);
    if (rc != 0) {
        return DK1_ERROR_IO;
    }
    impl->loop_thread_running = true;

    // Wait for the HID thread to publish its run loop. We cannot schedule
    // the device until that has happened, otherwise reports may be dropped.
    pthread_mutex_lock(&impl->ready_mutex);
    while (!impl->run_loop_ready) {
        pthread_cond_wait(&impl->ready_cond, &impl->ready_mutex);
    }
    pthread_mutex_unlock(&impl->ready_mutex);

    // Schedule the device on the HID thread's run loop and register the
    // input report callback. The callback uses impl->input_report_buf,
    // which is owned by the impl and must outlive the run loop.
    IOHIDDeviceScheduleWithRunLoop(impl->device, impl->run_loop, kCFRunLoopDefaultMode);
    IOHIDDeviceRegisterInputReportCallback(
        impl->device,
        impl->input_report_buf,
        DK1_INPUT_REPORT_BUF_LEN,
        hid_input_report_callback_normalized,
        impl
    );

    return DK1_OK;
}

static void mac_stop(DK1HIDBackend *backend) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->loop_thread_running) return;

    if (impl->run_loop) {
        CFRunLoopStop(impl->run_loop);
    }
    pthread_join(impl->loop_thread, NULL);
    impl->loop_thread_running = false;

    if (impl->run_loop) {
        CFRelease(impl->run_loop);
        impl->run_loop = NULL;
    }
}

static int mac_get_feature_report(
    DK1HIDBackend *backend,
    uint8_t report_id,
    uint8_t *buffer,
    size_t length
) {
    if (!backend || !buffer || length == 0) return DK1_ERROR_INVALID_ARGUMENT;
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device || !impl->device_opened) {
        return DK1_ERROR_NOT_OPEN;
    }

    // IOHIDDeviceGetReport uses the report ID in its argument and does not
    // require it to also be encoded as buffer[0]. The DK1 keepalive uses
    // report ID 8; the caller passes it explicitly via `report_id`.
    CFIndex reportLength = (CFIndex)length;
    IOReturn rc = IOHIDDeviceGetReport(
        impl->device,
        kIOHIDReportTypeFeature,
        (CFIndex)report_id,
        buffer,
        &reportLength
    );
    if (rc != kIOReturnSuccess) {
        return DK1_ERROR_IO;
    }
    return DK1_OK;
}

static int mac_set_feature_report(
    DK1HIDBackend *backend,
    const uint8_t *buffer,
    size_t length
) {
    if (!backend || !buffer || length == 0) return DK1_ERROR_INVALID_ARGUMENT;
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device || !impl->device_opened) {
        return DK1_ERROR_NOT_OPEN;
    }

    // The report ID is the first byte of the feature report payload, per
    // the DK1 firmware specification. We pull it out and hand the body
    // (without the ID prefix) to IOHIDDeviceSetReport.
    uint8_t report_id = buffer[0];
    const uint8_t *body = buffer + 1;
    size_t body_len = length - 1;

    CFIndex reportLength = (CFIndex)body_len;
    IOReturn rc = IOHIDDeviceSetReport(
        impl->device,
        kIOHIDReportTypeFeature,
        (CFIndex)report_id,
        (uint8_t *)body,
        reportLength
    );
    if (rc != kIOReturnSuccess) {
        return DK1_ERROR_IO;
    }
    return DK1_OK;
}

static void mac_set_raw_report_callback(
    DK1HIDBackend *backend,
    void (*callback)(const uint8_t *data, size_t length, void *user_data),
    void *user_data
) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl) return;
    impl->report_cb = callback;
    impl->user_data = user_data;
}

// IOHID input report callback. Runs on the HID thread's run loop. Keep it
// minimal: just hand the buffer to the registered callback. No allocation,
// no I/O, no logging.
static void hid_input_report_callback(
         void *context,
         IOReturn result,
         void *sender,
         IOHIDReportType type,
         uint32_t reportID,
         uint8_t *report,
         CFIndex reportLength
     )
{
    (void)result;
    (void)sender;
    (void)type;

    MacHIDImpl *impl = (MacHIDImpl *)context;
    if (!impl || !impl->report_cb) return;

    /* Forward the raw report to the registered callback. The library
    guarantees that the callback receives a normalized 62‑byte
    report, so no further processing is required here. */
    // Debug: confirm callback invocation
    fprintf(stderr, "[DEBUG] HID report received: len=%lld id=%d\n", (long long)reportLength, (int)reportID);
    impl->report_cb(report, (size_t)reportLength, impl->user_data);
}
/*
 * Factory: set up the function pointers on the DK1HIDBackend struct.
 * The implementation mirrors the one that was generated for the distribution
 * package.  The function is intentionally very small – all of the heavy
 * lifting happens in the static helper functions above.  We zero the
 * structure to avoid any stray data and expose the platform‑specific
 * operations.
 */
int dk1_hid_backend_create_mac(DK1HIDBackend *backend) {
    if (!backend) return DK1_ERROR_INVALID_ARGUMENT;
    memset(backend, 0, sizeof(*backend));
    backend->open                  = mac_open;
    backend->close                 = mac_close;
    backend->is_open               = mac_is_open;
    backend->start                 = mac_start;
    backend->stop                  = mac_stop;
    backend->get_feature_report    = mac_get_feature_report;
    backend->set_feature_report    = mac_set_feature_report;
    backend->set_raw_report_callback = mac_set_raw_report_callback;
    return DK1_OK;
}
