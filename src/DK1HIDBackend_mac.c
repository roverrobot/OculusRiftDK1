#include "DK1HIDBackend.h"
#include "DK1Tracker/DK1Error.h"
#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/hid/IOHIDDevice.h>
#include <IOKit/hid/IOHIDKeys.h>
#include <IOKit/IOMessage.h>
#include <CoreFoundation/CoreFoundation.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct {
    IOHIDManagerRef manager;
    IOHIDDeviceRef device;
    pthread_t loop_thread;
    bool running;
    void (*report_cb)(const uint8_t *data, size_t length, void *user_data);
    void *user_data;
    uint8_t *input_report_buf;
    CFIndex input_report_len;
} MacHIDImpl;

static void hid_report_callback(void *context, IOReturn result, void *sender, IOHIDReportType type, uint32_t reportID, uint8_t *report, CFIndex length) {
    MacHIDImpl *impl = (MacHIDImpl *)context;
    if (impl && impl->report_cb) {
        impl->report_cb(report, (size_t)length, impl->user_data);
    }
}

static void *run_loop_thread(void *arg) {
    MacHIDImpl *impl = (MacHIDImpl *)arg;
    CFRunLoopRun();
    return NULL;
}

static int mac_open(DK1HIDBackend *backend, uint16_t vid, uint16_t pid) {
    MacHIDImpl *impl = calloc(1, sizeof(MacHIDImpl));
    if (!impl) return DK1_ERROR_IO;

    impl->manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!impl->manager) {
        free(impl);
        return DK1_ERROR_IO;
    }
    
    CFMutableDictionaryRef match = CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    
    int v_id = vid;
    int p_id = pid;
    CFNumberRef v_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &v_id);
    CFNumberRef p_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &p_id);
    
    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey), v_num);
    CFDictionarySetValue(match, CFSTR(kIOHIDProductIDKey), p_num);
    
    IOHIDManagerSetDeviceMatching(impl->manager, match);
    
    CFRelease(v_num);
    CFRelease(p_num);
    CFRelease(match);

    if (IOHIDManagerOpen(impl->manager, kIOHIDOptionsTypeNone) != kIOReturnSuccess) {
        CFRelease(impl->manager);
        free(impl);
        return DK1_ERROR_OPEN_FAILED;
    }

    // Allocate buffer for input reports
    impl->input_report_buf = malloc(64);
    impl->input_report_len = 64;
    if (!impl->input_report_buf) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl);
        return DK1_ERROR_IO;
    }

    // Use IOHIDManagerCopyDevices to obtain a set of matching devices.
    CFSetRef device_set = IOHIDManagerCopyDevices(impl->manager);
    if (!device_set) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }

    CFIndex device_count = CFSetGetCount(device_set);
    if (device_count == 0) {
        CFRelease(device_set);
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }

    // Grab the first device from the set.
    impl->device = (IOHIDDeviceRef)CFSetGetValue(device_set, 0);
    if (!impl->device) {
        CFRelease(device_set);
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
        free(impl->input_report_buf);
        free(impl);
        return DK1_ERROR_NOT_FOUND;
    }
    CFRelease(device_set);

    backend->impl = impl;
    return DK1_OK;
}

static void mac_close(DK1HIDBackend *backend) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl) return;

    if (impl->running) {
        // In a real impl, we'd use a handle to the RunLoop
        pthread_cancel(impl->loop_thread);
        pthread_join(impl->loop_thread, NULL);
    }

    if (impl->device) CFRelease(impl->device);
    if (impl->manager) {
        IOHIDManagerClose(impl->manager, kIOHIDOptionsTypeNone);
        CFRelease(impl->manager);
    }
    if (impl->input_report_buf) free(impl->input_report_buf);
    free(impl);
    backend->impl = NULL;
}

static int mac_is_open(const DK1HIDBackend *backend) {
    return backend->impl ? DK1_OK : DK1_ERROR_NOT_OPEN;
}

static int mac_start(DK1HIDBackend *backend) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device) return DK1_ERROR_NOT_OPEN;

    // Register input report callback; use the preallocated buffer.
    IOHIDDeviceRegisterInputReportCallback(impl->device, impl->input_report_buf, impl->input_report_len, hid_report_callback, impl);
    
    impl->running = true;
    pthread_create(&impl->loop_thread, NULL, run_loop_thread, impl);
    
    return DK1_OK;
}

static void mac_stop(DK1HIDBackend *backend) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->running) return;
    
    impl->running = false;
}

static int mac_get_feature_report(DK1HIDBackend *backend, uint8_t report_id, uint8_t *buffer, size_t length) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device) return DK1_ERROR_IO;
    
    CFIndex lenVar = length;
    CFIndex len = IOHIDDeviceGetReport(impl->device, kIOHIDReportTypeFeature, report_id, buffer, &lenVar);
    return (lenVar > 0) ? DK1_OK : DK1_ERROR_IO;
}

static int mac_set_feature_report(DK1HIDBackend *backend, const uint8_t *buffer, size_t length) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl || !impl->device) return DK1_ERROR_IO;
    
    CFIndex lenVar = length;
    CFIndex len = IOHIDDeviceSetReport(impl->device, kIOHIDReportTypeFeature, 0, buffer, lenVar);
    return (lenVar > 0) ? DK1_OK : DK1_ERROR_IO;
}

static void mac_set_raw_report_callback(DK1HIDBackend *backend, void (*callback)(const uint8_t *data, size_t length, void *user_data), void *user_data) {
    MacHIDImpl *impl = (MacHIDImpl *)backend->impl;
    if (!impl) return;
    impl->report_cb = callback;
    impl->user_data = user_data;
}

int dk1_hid_backend_create_mac(DK1HIDBackend *backend) {
    backend->open = mac_open;
    backend->close = mac_close;
    backend->is_open = mac_is_open;
    backend->start = mac_start;
    backend->stop = mac_stop;
    backend->get_feature_report = mac_get_feature_report;
    backend->set_feature_report = mac_set_feature_report;
    backend->set_raw_report_callback = mac_set_raw_report_callback;
    return DK1_OK;
}
