#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/split/bluetooth/peripheral.h>
#include <zmk/usb.h>

#if __has_include(<zmk/split/central.h>)
#include <zmk/split/central.h>
#else
#include <zmk/split/bluetooth/central.h>
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(led_widget_led)),
             "No node labelled led_widget_led for LED_WIDGET");

static const struct device *led_dev = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(led_widget_led)));
static const uint32_t led_idx = DT_NODE_CHILD_IDX(DT_NODELABEL(led_widget_led));

// log shorthands
#define LOG_CONN_CENTRAL(index, status)                                               \
    LOG_INF("Profile %d %s", index, status)
#define LOG_CONN_PERIPHERAL(status)                                                   \
    LOG_INF("Peripheral %s", status)
#define LOG_BATTERY(battery_level)                                                    \
    LOG_INF("Battery level %d", battery_level)

enum message_type {
    MESSAGE_COLOR_SET,
    MESSAGE_PATTERN_SWAP,
};

enum color {
    COLOR_OFF = 0,
    COLOR_ON,
};

struct pattern {
    uint8_t times;
    uint16_t duration_ms;
    uint16_t sleep_ms;
};

enum pattern_type {
    // lowest pri
    PATTERN_UNKNOWN = -1,
    PATTERN_BATT_30,
    PATTERN_BATT_20,
    PATTERN_BATT_10,
    PATTERN_ADVERTISING,
    PATTERN_CONNECTED,
    // highest pri
};

// must match the order of the patterns in the enum above
static const struct pattern PATTERNS[] = {
    // PATTERN_BATT_30
    {
        .times = 3,
        .duration_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_MS,
        .sleep_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_SLEEP_MS,
    },
    // PATTERN_BATT_20
    {
        .times = 2,
        .duration_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_MS,
        .sleep_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_SLEEP_MS,
    },
    // PATTERN_BATT_10
    {
        .times = 1,
        .duration_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_MS,
        .sleep_ms = CONFIG_LED_WIDGET_BATTERY_BLINK_SLEEP_MS,
    },
    // PATTERN_ADVERTISING
    {
        .times = 1,
        .duration_ms = CONFIG_LED_WIDGET_CONN_ADVERTISING_MS,
        .sleep_ms = 0,
    },
    // PATTERN_CONNECTED
    {
        .times = 1,
        .duration_ms = CONFIG_LED_WIDGET_CONN_CONNECTED_MS,
        .sleep_ms = 0,
    },
};

struct message_item {
    enum message_type type;
    union {
        enum color color;
        struct {
            enum pattern_type pattern_off;
            enum pattern_type pattern_on;
        };
    };
};

// flag to indicate whether the initial boot up sequence is complete
static bool initialized = false;

// track current color to prevent unnecessary calls (TODO: is this necessary?)
enum color led_current_color = COLOR_OFF;

// low-level method to control the LED
static void set_led(enum color color, uint16_t duration_ms) {
    if (led_current_color != color) {
        if (color) {
            led_on(led_dev, led_idx);
        } else {
            led_off(led_dev, led_idx);
        }

        led_current_color = color;
    }
    if (duration_ms > 0) {
        k_sleep(K_MSEC(duration_ms));
    }
}

// define message queue of blink work items, that will be processed by a
// separate thread
K_MSGQ_DEFINE(led_msgq, sizeof(struct message_item), 16, 1);

bool usb_current_powered = false;

static void indicate_usb_powered(void) {
    struct message_item msg = {.type = MESSAGE_COLOR_SET};
    bool powered = zmk_usb_is_powered();
    if (usb_current_powered != powered) {
        msg.color = powered ? COLOR_ON : COLOR_OFF;
        k_msgq_put(&led_msgq, &msg, K_NO_WAIT);

        if (powered) {
            LOG_INF("USB powered, set led on");
        } else {
            LOG_INF("USB not powered, set led off");
        }

        usb_current_powered = powered;
    }
}

static int led_charge_listener_cb(const zmk_event_t *eh) {
    if (initialized) {
        indicate_usb_powered();
    }

    return 0;
}

// run led_charge_listener_cb on usb state change event
ZMK_LISTENER(led_charge_listener, led_charge_listener_cb);
ZMK_SUBSCRIPTION(led_charge_listener, zmk_usb_conn_state_changed);

enum pattern_type current_connectivity_pattern = PATTERN_UNKNOWN;

static void indicate_connectivity_internal(void) {
    struct message_item msg = {.type = MESSAGE_PATTERN_SWAP};
    enum pattern_type next_connectivity_pattern;

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    switch (zmk_endpoints_selected().transport) {
    case ZMK_TRANSPORT_USB:
    default: // ZMK_TRANSPORT_BLE
#if IS_ENABLED(CONFIG_ZMK_BLE)
        uint8_t profile_index = zmk_ble_active_profile_index();
        if (zmk_ble_active_profile_is_connected()) {
            LOG_CONN_CENTRAL(profile_index, "connected");
            next_connectivity_pattern = PATTERN_CONNECTED;
        } else if (zmk_ble_active_profile_is_open()) {
            LOG_CONN_CENTRAL(profile_index, "open");
            next_connectivity_pattern = PATTERN_ADVERTISING;
        } else {
            LOG_CONN_CENTRAL(profile_index, "not connected");
            next_connectivity_pattern = PATTERN_UNKNOWN;
        }
#endif
        break;
    }
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    if (zmk_split_bt_peripheral_is_connected()) {
        LOG_CONN_PERIPHERAL("connected");
        next_connectivity_pattern = PATTERN_CONNECTED;
    } else {
        LOG_CONN_PERIPHERAL("not connected");
        next_connectivity_pattern = PATTERN_UNKNOWN;
    }
#endif

    if (current_connectivity_pattern != next_connectivity_pattern ) {
        msg.pattern_off = current_connectivity_pattern;
        msg.pattern_on = next_connectivity_pattern;
        k_msgq_put(&led_msgq, &msg, K_NO_WAIT);

        // only blink the connected pattern once
        if (next_connectivity_pattern == PATTERN_CONNECTED) {
            msg.pattern_off = next_connectivity_pattern;
            msg.pattern_on = PATTERN_UNKNOWN;
            k_msgq_put(&led_msgq, &msg, K_NO_WAIT);
            next_connectivity_pattern = PATTERN_UNKNOWN;
        }

        current_connectivity_pattern = next_connectivity_pattern;
    }

}

// debouncing to ignore all but last connectivity event, to prevent repeat blinks
static struct k_work_delayable indicate_connectivity_work;
static void indicate_connectivity_cb(struct k_work *work) { indicate_connectivity_internal(); }
static void indicate_connectivity(void) { k_work_reschedule(&indicate_connectivity_work, K_MSEC(16)); }

static int led_output_listener_cb(const zmk_event_t *eh) {
    if (initialized) {
        indicate_connectivity();
    }
    return 0;
}

ZMK_LISTENER(led_output_listener, led_output_listener_cb);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
// run led_output_listener_cb on BLE profile change (on central)
#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(led_output_listener, zmk_ble_active_profile_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BLE)
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
// run led_output_listener_cb on peripheral status change event
ZMK_SUBSCRIPTION(led_output_listener, zmk_split_peripheral_status_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
enum pattern_type current_battery_pattern = PATTERN_UNKNOWN;

static void set_battery_level(uint8_t battery_level) {
    struct message_item msg = {.type = MESSAGE_PATTERN_SWAP};
    enum pattern_type next_battery_pattern;

    if (battery_level == 0) {
        LOG_INF("Battery level undetermined (zero)");
        return;
    }

    LOG_BATTERY(battery_level);
    if (battery_level <= 10) {
        next_battery_pattern = PATTERN_BATT_10;
    } else if (battery_level <= 20) {
        next_battery_pattern = PATTERN_BATT_20;
    } else if (battery_level <= 30) {
        next_battery_pattern = PATTERN_BATT_30;
    } else {
        next_battery_pattern = PATTERN_UNKNOWN;
    }

    if (current_battery_pattern != next_battery_pattern) {
        msg.pattern_off = current_battery_pattern;
        msg.pattern_on = next_battery_pattern;
        k_msgq_put(&led_msgq, &msg, K_NO_WAIT);

        current_battery_pattern = next_battery_pattern;
    }
}

static void indicate_battery(void) {
    int retry = 0;

    uint8_t battery_level = zmk_battery_state_of_charge();
    while (battery_level == 0 && retry++ < 10) {
        k_sleep(K_MSEC(100));
        battery_level = zmk_battery_state_of_charge();
    };

    set_battery_level(battery_level);
}

static int led_battery_listener_cb(const zmk_event_t *eh) {
    if (initialized) {
        uint8_t battery_level = as_zmk_battery_state_changed(eh)->state_of_charge;
        set_battery_level(battery_level);
    }

    return 0;
}

// run led_battery_listener_cb on battery state change event
ZMK_LISTENER(led_battery_listener, led_battery_listener_cb);
ZMK_SUBSCRIPTION(led_battery_listener, zmk_battery_state_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

// default color to use when no patterns are active
enum color led_default_color = COLOR_OFF;

static void display_pattern(uint8_t pattern_index) {
    if (pattern_index >= sizeof(PATTERNS) / sizeof(PATTERNS[0])) {
        LOG_WRN("Invalid pattern index %d", pattern_index);
        return;
    }

    const struct pattern *p = &PATTERNS[pattern_index];
    for (uint8_t i = 0; i < p->times; i++) {
        set_led(led_default_color == COLOR_ON ? COLOR_OFF : COLOR_ON, p->duration_ms);
        if (i < p->times - 1) {
            set_led(led_default_color, p->sleep_ms);
        }
    }
    set_led(led_default_color, CONFIG_LED_WIDGET_INTERVAL_MS);
}

// track currently enabled patterns as a bitmask
uint8_t led_current_patterns = 0;

extern void led_process_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    k_work_init_delayable(&indicate_connectivity_work, indicate_connectivity_cb);

    set_led(led_default_color, 0);

    while (true) {
        // wait until a message is received and process it
        struct message_item msg;
        k_msgq_get(&led_msgq, &msg, led_current_patterns == 0 ? K_FOREVER : K_NO_WAIT);
        switch (msg.type) {
        case MESSAGE_COLOR_SET:
            LOG_DBG("Got a layer color item from msgq, color %d", msg.color);
            led_default_color = msg.color;
            break;
        case MESSAGE_PATTERN_SWAP:
            if (msg.pattern_off != PATTERN_UNKNOWN) {
                led_current_patterns &= ~(1 << msg.pattern_off);
            }
            if (msg.pattern_on != PATTERN_UNKNOWN) {
                led_current_patterns |= (1 << msg.pattern_on);
            }
            LOG_DBG(
                "Got a pattern swap item from msgq, pattern off %d, pattern on %d, current pattern %d",
                msg.pattern_off,
                msg.pattern_on,
                led_current_patterns
            );
            break;
        default:
            LOG_WRN("Unknown message type %d", msg.type);
            break;
        }

        if (led_current_patterns == 0) {
            set_led(led_default_color, 0);
            continue;
        }

        uint8_t highest_priority_pattern;
        uint8_t v = led_current_patterns >> 1;
        for (highest_priority_pattern = 0; v; highest_priority_pattern++) {
            v >>= 1;
        }

        display_pattern(highest_priority_pattern);
    }
}

// define led_process_thread with stack size 1024, start running it 100 ms after
// boot
K_THREAD_DEFINE(led_process_tid, 1024, led_process_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);

extern void led_init_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    indicate_usb_powered();

    // check and indicate current profile or peripheral connectivity status
    LOG_INF("Indicating initial connectivity status");
    indicate_connectivity();

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    // check and indicate battery level on thread start
    LOG_INF("Indicating initial battery status");

    indicate_battery();
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

    initialized = true;
    LOG_INF("Finished initializing LED widget");
}

// run init thread on boot for initial battery+output checks
K_THREAD_DEFINE(led_init_tid, 1024, led_init_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 200);
