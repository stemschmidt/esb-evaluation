target_sources(app PRIVATE 
    src/main.c
    src/feedback_nrf.c
    ../../zephyr/samples/subsys/usb/common/sample_usbd_init.c
)
