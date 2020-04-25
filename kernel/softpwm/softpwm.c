#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>

/* Module info */
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_AUTHOR("Ethan Takla");
MODULE_DESCRIPTION("A software PWM driver");

/* ioctl device vairables*/
dev_t dev;
static struct class *devClass;
static struct cdev softpwmCdev;

/* PWM state variables */
atomic_t dutyCycle;
static int pinState;

/* Timer definitions */
static struct hrtimer timer;
static ktime_t timerPeriod;
atomic_t lastUpdateTimeMS;

/* Constant definitions */
#define MOTOR_PWM_PIN 468
#define PWM_PERIOD_US 2000
#define DUTY_CYCLE_MAX 1000
#define CONTROL_TIMEOUT_MS 1000
#define DUTY_CYCLE_MAX_JITTER_US 1

/* Function declarations */
static int softPWMLoop(void *);
static void softPWMCleanup(void);
static enum hrtimer_restart checkTimeout(struct hrtimer*);
static long ioctlETX(struct file *, unsigned int, unsigned long);

/* IOCTL definitions */
#define WR_VALUE _IOW('a', 'a', int32_t *)

/* Task definitions */
static struct task_struct *softPWMThread;

/* Device file struct */
static struct file_operations fops =
{
        .owner = THIS_MODULE,
        .unlocked_ioctl = ioctlETX
};

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Initializes the soft PWM kernel module, including setting up the   #
#                softpwm_device, initializing the GPIO, and starting the control    #
#                loop.                                                              #
#                                                                                   #
************************************************************************************/

static int __init initSoftPWM(void)
{
    // Initialize the state of the pin
    pinState = 0;

    // Set the duty cycle to zero initially
    atomic_set(&dutyCycle, 0);

    // Set the last update time to now
    atomic_set(&lastUpdateTimeMS, ktime_get().tv64/1000000);

    // Allocate major number
    alloc_chrdev_region(&dev, 0, 1, "softpwm");

    // Create cdev structure
    cdev_init(&softpwmCdev, &fops);

    // Add the char device to the system
    cdev_add(&softpwmCdev, dev, 1);

    // Create the struct class
    devClass = class_create(THIS_MODULE, "softpwm_class");

    // Create the device
    device_create(devClass, NULL, dev, NULL, "softpwm_device");

    // Set up the GPIO
    gpio_request(MOTOR_PWM_PIN, "sysfs");
    gpio_direction_output(MOTOR_PWM_PIN, pinState);
    gpio_export(MOTOR_PWM_PIN, false);

    // Set up the timer w/interrupts
    timerPeriod = ktime_set(0, CONTROL_TIMEOUT_MS*1000000);
    hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    timer.function = checkTimeout;
    hrtimer_start(&timer, timerPeriod, HRTIMER_MODE_REL);

    // Create the soft PWM thread
    softPWMThread = kthread_create(softPWMLoop, NULL, "softpwm");

    // Wake it up if it was successfully created
    if (softPWMThread)
        wake_up_process(softPWMThread);

    return 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @args                 Arguments sent from thread initialization         #
#                                                                                   #
#   Description: A kthread that responsible for generatung the PWM signal based     #
#                on the duty cycle.                                                 #
#                                                                                   #
************************************************************************************/

static int softPWMLoop(void *args)
{

    while (!kthread_should_stop())
    {
        // Compute the on time
        u32 onTime = (atomic_read(&dutyCycle) * PWM_PERIOD_US) / DUTY_CYCLE_MAX;

        // Set pin state and delay appropriatly
        if (atomic_read(&dutyCycle) != 0 && atomic_read(&dutyCycle) <= DUTY_CYCLE_MAX)
        {
            // Set the pin state
            gpio_set_value(MOTOR_PWM_PIN, pinState);

            if (pinState)
            {
                usleep_range(onTime - DUTY_CYCLE_MAX_JITTER_US, onTime + DUTY_CYCLE_MAX_JITTER_US);
            }
            else
            {
                usleep_range(PWM_PERIOD_US - onTime - DUTY_CYCLE_MAX_JITTER_US,
                             PWM_PERIOD_US - onTime + DUTY_CYCLE_MAX_JITTER_US);
            }

            pinState ^= 1;
        }
        else
        {
            gpio_set_value(MOTOR_PWM_PIN, 0);
            usleep_range(PWM_PERIOD_US - DUTY_CYCLE_MAX_JITTER_US, 
                         PWM_PERIOD_US + DUTY_CYCLE_MAX_JITTER_US);
        }
        
    }

    return 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Cleans up all resources used by the module.                        #
#                                                                                   #
************************************************************************************/

static void softPWMCleanup(void)
{
    // Kill the thread
    kthread_stop(softPWMThread);

    // Free ioctl resources
    device_destroy(devClass, dev);
    class_destroy(devClass);
    cdev_del(&softpwmCdev);
    unregister_chrdev_region(dev, 1);

    // Free the GPIO resources
    gpio_set_value(MOTOR_PWM_PIN, 0);
    gpio_unexport(MOTOR_PWM_PIN);
    gpio_free(MOTOR_PWM_PIN);

    // Free the timer resources
    hrtimer_cancel(&timer);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @file                 The ioctl file struct for the device              #
#           @cmd                  The type of ioctl command                         #
#           @arg                  Argument passed from the ioctl call               #
#                                                                                   #
#   Description: Handles a ioctl call to write to the device, and atomically sets   #
#                the duty cycle based on the value.                                 #
#                                                                                   #
************************************************************************************/

static long ioctlETX(struct file *file, unsigned int cmd, unsigned long arg)
{
    u32 dutyCycleTmp;

    if (cmd == WR_VALUE)
    {
        if(!copy_from_user(&dutyCycleTmp, (u32 *)arg, sizeof(dutyCycleTmp)))
        {
            atomic_set(&dutyCycle, dutyCycleTmp);
            atomic_set(&lastUpdateTimeMS, (ktime_get().tv64/1000000));
        }
    }

    return 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @tm                   Pointer to the timer struct                       #
#                                                                                   #
#   Description: Periodically checks to see the last time the duty cycle was        #
#                updated. If it exceeds a timeout, it sets the duty cycle to zero.  #
#                                                                                   #
************************************************************************************/

static enum hrtimer_restart checkTimeout(struct hrtimer * tm)
{
    // Set the duty cycle to zero if we haven't received a command in a while
    if((ktime_get().tv64/1000000)-atomic_read(&lastUpdateTimeMS) > CONTROL_TIMEOUT_MS)
        atomic_set(&dutyCycle, 0);
    
    // Update the timer period
    hrtimer_forward_now(tm, timerPeriod);

    return HRTIMER_RESTART;
}

module_init(initSoftPWM);
module_exit(softPWMCleanup);