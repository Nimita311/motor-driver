/**
 * @file     app.h
 * @brief    Application code interface.
 * @author   Haoze Zhang
 * @version  20200211
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
  * Initialize the application.
  */
void appInit();

/**
  * Create all application level FreeRTOS tasks.
  * This function should be called in main before the kernel starts.
  * @return FreeRTOS success flag
  */
BaseType_t appCreateTasks();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* INC_APP_H_ */
