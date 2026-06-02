#ifndef DK1_HID_BACKEND_H
#define DK1_HID_BACKEND_H

#include <stdint.h>
#include <stddef.h>

typedef struct DK1HIDBackend DK1HIDBackend;

struct DK1HIDBackend {
    void *impl;

    int  (*open)(DK1HIDBackend *backend, uint16_t vid, uint16_t pid);
    void (*close)(DK1HIDBackend *backend);
    int  (*is_open)(const DK1HIDBackend *backend);

    int  (*start)(DK1HIDBackend *backend);
    void (*stop)(DK1HIDBackend *backend);

    int  (*get_feature_report)(
        DK1HIDBackend *backend,
        uint8_t report_id,
        uint8_t *buffer,
        size_t length
    );

    int  (*set_feature_report)(
        DK1HIDBackend *backend,
        const uint8_t *buffer,
        size_t length
    );

    void (*set_raw_report_callback)(
        DK1HIDBackend *backend,
        void (*callback)(const uint8_t *data, size_t length, void *user_data),
        void *user_data
    );
};

int dk1_hid_backend_create_mac(DK1HIDBackend *backend);

#endif
