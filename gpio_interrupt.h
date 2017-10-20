/*
 * gpio_interrupt.h
 *
 *  Created on: 22-09-2015
 *      Author: Sang
 */

#ifndef GPIO_INTERRUPT_H_
#define GPIO_INTERRUPT_H_

void config_gpio(void);
void config_first_interrupt(void);
void main_receiver_config(void);
void config_receiver(int num);
void enable_receiver(int num);
void clear_all(void);
void start_counter(void);

#endif /* GPIO_INTERRUPT_H_ */
