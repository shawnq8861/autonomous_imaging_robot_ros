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
MODULE_DESCRIPTION("Hall effect sensor driver");

/* ioctl device vairables*/
dev_t dev;
static struct class *devClass;
static struct cdev halleffectCdev;

/* Hall effect state variables */
atomic_t activated;
atomic_t hallEffectCount;

/* Interrupt variables */
static unsigned int irqNumber;

/* Timer definitions */
atomic_t lastUpdateTimeMS;
static ktime_t timerPeriod;
static struct hrtimer timer;

/* Constant definitions */
#define HALL_EFFECT_PIN 433
#define ACTIVATED_MIN_COUNT 2
#define DEBOUNCE_PERIOD_MS 500
#define STATE_EXPIRATION_POLL_PERIOD_NS 10000000

/* Function declarations */
static enum hrtimer_restart checkTimeout(struct hrtimer*);
static long ioctlETX(struct file *, unsigned int, unsigned long);
static irq_handler_t halleffect_irq_handler(unsigned int, void*, struct pt_regs*);

/* IOCTL definitions */
#define WR_VALUE _IOW('a', 'a', int32_t *)
#define RD_VALUE _IOR('a','b',int32_t*)

/* Device file struct */
static struct file_operations fops =
    {
        .owner = THIS_MODULE,
        .unlocked_ioctl = ioctlETX};

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Initializes the hall effect kernel module, including setting up    #
#                the initializing the GPIO, setting up the character device, and    #
#                configuring interrupts.                                            #
#                                                                                   #
************************************************************************************/

static int __init initHallEffect(void)
{
    // Set the initial state to the hall effect not being activated
    atomic_set(&activated, 0);
    atomic_set(&hallEffectCount, 0);
    
    // Allocate major number
    alloc_chrdev_region(&dev, 0, 1, "halleffect");

    // Create cdev structure
    cdev_init(&halleffectCdev, &fops);

    // Add the char device to the system
    cdev_add(&halleffectCdev, dev, 1);

    // Create the struct class
    devClass = class_create(THIS_MODULE, "halleffect_class");

    // Create the device
    device_create(devClass, NULL, dev, NULL, "halleffect_device");

    // Set up the GPIO
    gpio_request(HALL_EFFECT_PIN, "sysfs");
    gpio_direction_input(HALL_EFFECT_PIN);
    gpio_export(HALL_EFFECT_PIN, false);

    // Translate the GPIO to interrupt number
    irqNumber = gpio_to_irq(HALL_EFFECT_PIN);

    // Request the interrupt
    request_irq(irqNumber, (irq_handler_t)halleffect_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "halleffect_gpio_handler",
                NULL);

    // Set up the timer w/interrupts
    timerPeriod = ktime_set(0, STATE_EXPIRATION_POLL_PERIOD_NS);
    hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    timer.function = checkTimeout;
    hrtimer_start(&timer, timerPeriod, HRTIMER_MODE_REL);

    return 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Cleans up all resources used by the module.                        #
#                                                                                   #
************************************************************************************/

static void hallEffectCleanup(void)
{
    // Free ioctl resources
    device_destroy(devClass, dev);
    class_destroy(devClass);
    cdev_del(&halleffectCdev);
    unregister_chrdev_region(dev, 1);

    // Free the GPIO resources
    gpio_unexport(HALL_EFFECT_PIN);
    gpio_free(HALL_EFFECT_PIN);

    // Free up the irq
    free_irq(irqNumber, NULL);

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
#   Description: Handles a ioctl calls to interface with the device. This includes  #
#                reading and setting the hall effect state.                         #
#                                                                                   #
************************************************************************************/

static long ioctlETX(struct file *file, unsigned int cmd, unsigned long arg)
{
    u32 activatedTmp;

    if (cmd == WR_VALUE)
    {
        // Set the activated variable to the desired state
        copy_from_user(&activatedTmp, (u32 *)arg, sizeof(activatedTmp));
        atomic_set(&activated, activatedTmp);
    }
    else if (cmd == RD_VALUE)
    {
        // Read the activated variable and copy it over to user space
        activatedTmp = atomic_read(&activated);
        copy_to_user((int32_t*) arg, &activatedTmp, sizeof(activatedTmp));
    }

    return 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @irq                  The interrupt #                                   #
#           @dev_id               ID of the device                                  #
#           @regs                 Pointer to the low-level registers                #
#                                                                                   #
#   Description: Interrupt handler that fires whenever the state of the hall        #
#                effect pin changes, and records when it is detected.               #
#                                                                                   #
************************************************************************************/

static irq_handler_t halleffect_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
    // Set the last update time
    atomic_set(&lastUpdateTimeMS, (ktime_get().tv64/1000000));

    // Increase the hall effect count
    atomic_set(&hallEffectCount, atomic_read(&hallEffectCount)+1);

    // Mark as activated if we reach a certain count
    if(atomic_read(&hallEffectCount) == ACTIVATED_MIN_COUNT)
        atomic_set(&activated, 1);

    return (irq_handler_t)IRQ_HANDLED;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @tm                   Pointer to the timer struct                       #
#                                                                                   #
#   Description: When this timer interrupt fires, it checks to see if the last      #
#                time a state was detected was within the debounce window. If it    #
#                is outside of the debounce window, it resets the count.            #
#                                                                                   #
************************************************************************************/

static enum hrtimer_restart checkTimeout(struct hrtimer * tm)
{
    // Set the duty cycle to zero if we haven't received a command in a while
    if((ktime_get().tv64/1000000)-atomic_read(&lastUpdateTimeMS) > DEBOUNCE_PERIOD_MS)
        atomic_set(&hallEffectCount, 0);
    
    // Update the timer period
    hrtimer_forward_now(tm, timerPeriod);
    
    return HRTIMER_RESTART;
}

module_init(initHallEffect);
module_exit(hallEffectCleanup);
