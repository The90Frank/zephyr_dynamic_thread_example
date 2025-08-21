/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>

/* size of stack area used by each thread */
#define STACKSIZE 512

/* scheduling priority used by each thread */
#define PRIORITY -3

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#if !DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(LED1_NODE)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(LED2_NODE)
#error "Unsupported board: led2 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(LED3_NODE)
#error "Unsupported board: led3 devicetree alias is not defined"
#endif

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
#if !DT_NODE_HAS_STATUS_OKAY(SW1_NODE)
#error "Unsupported board: sw1 devicetree alias is not defined"
#endif

/****************BUTTONS******************/

volatile sig_atomic_t button_state = 1;
void button0_pressed(const struct device *dev, struct gpio_callback *cb,
                     uint32_t pins);
void button1_pressed(const struct device *dev, struct gpio_callback *cb,
                     uint32_t pins);

typedef struct
{
   struct gpio_dt_spec spec;
   struct gpio_callback cb_data;
   gpio_callback_handler_t pressed;
} button_t;

button_t buttons[] = {
    {.spec = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}), .pressed = button0_pressed},
    {.spec = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0}), .pressed = button1_pressed}};

/****************SERIAL*******************/

struct printk_data_t
{
   void *fifo_reserved; /* 1st word reserved for use by fifo */
   uint32_t led;
   uint32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

void uart_out(void);

K_THREAD_DEFINE(uart_out_id, STACKSIZE, uart_out, NULL, NULL, NULL,
                PRIORITY, 0, 0);

/*****************LED*******************/

typedef struct _led_t
{
   struct gpio_dt_spec spec;
   uint8_t num;
   uint32_t sleep_ms;
   k_thread_entry_t func;
} led_t;

void blink(void *, void *, void *);

led_t leds[] = {
    {.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}), .num = 0, .sleep_ms = 1000, .func = blink},
    {.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}), .num = 1, .sleep_ms = 822, .func = blink},
    {.spec = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0}), .num = 2, .sleep_ms = 266, .func = blink},
    {.spec = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios, {0}), .num = 3, .sleep_ms = 555, .func = blink},
};

k_tid_t led_thread_ids[4];
k_thread_stack_t *led_stack[4];
struct k_thread led_threads[4];

int main(void)
{
   int ret = 0;

   for (int i = 0; i < ARRAY_SIZE(buttons); i++)
   {
      if (!gpio_is_ready_dt(&(buttons[i].spec)))
      {
         printk("Error: button%d device %s is not ready\n",
                i, buttons[i].spec.port->name);
         ret = -ENODEV;
      }
      else if (!buttons[i].pressed)
      {
         printk("Error: button%d pressed callback is NULL\n", i);
         ret = -EFAULT;
      }
      else
      {
         ret = gpio_pin_configure_dt(&(buttons[i].spec), GPIO_INPUT);
         if (ret != 0)
         {
            printk("Error %d: button%d failed to configure %s pin %d\n",
                   ret, i, buttons[i].spec.port->name, buttons[i].spec.pin);
         }
      }

      if (!ret)
      {
         ret = gpio_pin_interrupt_configure_dt(&(buttons[i].spec),
                                               GPIO_INT_EDGE_TO_ACTIVE);
         if (ret != 0)
         {
            printk("Error %d: button%d failed to configure interrupt on %s pin %d\n",
                   ret, i, buttons[i].spec.port->name, buttons[i].spec.pin);
         }
      }

      if (!ret)
      {
         gpio_init_callback(&(buttons[i].cb_data), buttons[i].pressed,
                            BIT(buttons[i].spec.pin));
         ret = gpio_add_callback(buttons[i].spec.port, &(buttons[i].cb_data));
         if (ret != 0)
         {
            printk("Error %d: failed to setup button interrupts\n", ret);
         }
         else
         {
            printk("Set up button%d at %s pin %d\n", i,
                   buttons[i].spec.port->name, buttons[i].spec.pin);
         }
      }
   }

   for (int i = 0; i < ARRAY_SIZE(leds); i++)
   {
      led_stack[i] = k_thread_stack_alloc(STACKSIZE, 0);
      if (led_stack[i] == NULL)
      {
         printk("Error: failed to allocate stack for LED %d\n", i);
         return -ENOMEM;
      }
      led_thread_ids[i] = k_thread_create(&led_threads[i], led_stack[i],
                                          STACKSIZE,
                                          leds[i].func, &(leds[i]), NULL, NULL,
                                          PRIORITY, 0, K_NO_WAIT);
      k_thread_start(&led_threads[i]);
   }

   return 0;
}

/***********************/

void button0_pressed(const struct device *dev, struct gpio_callback *cb,
                     uint32_t pins)
{
   printk("Button 0 pressed\n");
   button_state = 1;
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
                     uint32_t pins)
{
   printk("Button 1 pressed\n");
   button_state = 0;
}

/************************/

void blink(void *p1, void *p2, void *p3)
{
   led_t *led = (led_t *)p1;
   printk("Led %d: spec.port=%p, spec.pin=%d, num=%d, sleep_ms=%d, func=%p\n",
          led->num, led->spec.port, led->spec.pin, led->num, led->sleep_ms, led->func);
   const struct gpio_dt_spec *spec = &led->spec;
   int cnt = 0;
   int ret;

   if (!device_is_ready(spec->port))
   {
      printk("Error: %s device is not ready\n", spec->port->name);
      return;
   }

   ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
   if (ret != 0)
   {
      printk("Error %d: failed to configure pin %d (LED '%d')\n",
             ret, spec->pin, led->num);
      return;
   }

   while (1)
   {
      if (button_state)
      {
         gpio_pin_set(spec->port, spec->pin, cnt % 2);
      }
      else
      {
         gpio_pin_set(spec->port, spec->pin, 0);
      }
      struct printk_data_t tx_data = {.led = led->num, .cnt = cnt};

      size_t size = sizeof(struct printk_data_t);
      char *mem_ptr = k_malloc(size);
      __ASSERT_NO_MSG(mem_ptr != 0);

      memcpy(mem_ptr, &tx_data, size);

      k_fifo_put(&printk_fifo, mem_ptr);

      k_msleep(led->sleep_ms);
      cnt++;
   }
}

/***********************/

void uart_out(void)
{
   while (1)
   {
      struct printk_data_t *rx_data = k_fifo_get(&printk_fifo,
                                                 K_FOREVER);
      printk("Toggled led%d; counter=%d\n",
             rx_data->led, rx_data->cnt);
      k_free(rx_data);
   }
}
