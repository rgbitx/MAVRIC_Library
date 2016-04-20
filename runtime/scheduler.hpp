/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file scheduler.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Scheduler
 *
 ******************************************************************************/


#ifndef SCHEDULER_HPP_
#define SCHEDULER_HPP_

#include <stdint.h>
#include "runtime/scheduler_task.hpp"


#define SchedulerIMEBASE 1000000  ///< time base for the scheduler


/**
 * \brief   Scheduler
 *
 */
class Scheduler
{

public:

    /**
     * \brief   Scheduling strategy
     */
    enum strategy_t
    {
        ROUND_ROBIN,                ///<    Round robin scheduling
        FIXED_PRIORITY              ///<    Fixed priority scheduling
    };


    /**
     * \brief Scheduler configuration
     */
    struct conf_t
    {
        uint32_t max_task_count;                    ///<    Maximum number of tasks
        Scheduler::strategy_t schedule_strategy;    ///<    Schedule strategy
        bool debug;                                 ///<    Indicates whether the scheduler should print debug messages
    };


    /**
     * \brief               Constructor
     *
     * \param   config      Configuration
     */
    Scheduler(const Scheduler::conf_t config = default_config());


    /**
     * \brief                   Register a new task to the task set, in the first available slot
     *
     * \param repeat_period     Repeat period (us)
     * \param run_mode          Run mode
     * \param timing_mode       Timing mode
     * \param priority          Priority
     * \param call_function     Function pointer to be called
     * \param function_argument Argument to be passed to the function
     * \param task_id           Unique task identifier
     *
     * \return                  True if the task was successfully added, False if not
     */
    bool add_task(uint32_t repeat_period, Scheduler_task::run_mode_t run_mode, Scheduler_task::timing_mode_t timing_mode, Scheduler_task::priority_t priority, Scheduler_task::task_function_t call_function, Scheduler_task::task_argument_t function_argument, uint32_t task_id);


    /**
     * \brief                Sort tasks by decreasing priority, then by increasing repeat period
     *
     * \return               True if the task was successfully sorted, False if not
     */
    bool sort_tasks();


    /**
     * \brief                Run update (check for tasks ready for execution and execute them)
     *
     * \return               Number of realtime violations
     */
    int32_t update();


    /**
     * \brief               Find a task according to its idea
     *
     * \param   task_id     ID of the target task
     *
     * \return              Pointer to the target task
     */
    Scheduler_task* get_task_by_id(uint16_t task_id) const;


    /**
     * \brief               Find a task according to its index
     *
     * \param task_index    Index of the target task
     *
     * \return              Pointer to the target task
     */
    Scheduler_task* get_task_by_index(uint16_t task_index) const;


    /**
     * \brief               Returns whether scheduler is in debug mode (printing messages)
     *
     * \param task_index    Index of the target task
     *
     * \return              Pointer to the target task
     */
    bool is_debug();


    /**
     * \brief           Suspends all tasks
     *
     * \param delay     Duration (us)
     */
    void suspend_all_tasks(uint32_t delay);


    /**
     * \brief       Run all tasks immediately
     *
     */
    void run_all_tasks_now();


    /**
     * \brief       Creates and returns default config for schedulers
     *
     * \return      default_config
     */
    static Scheduler::conf_t default_config();


private:
    strategy_t schedule_strategy;               ///<    Scheduling strategy
    bool debug;                                 ///<    Indicates whether the scheduler should print debug messages
    uint32_t task_count;                        ///<    Number_of_tasks
    uint32_t max_task_count;                    ///<    Maximum number of tasks
    uint32_t current_schedule_slot;             ///<    Slot of the task being executed
    Scheduler_task* tasks;                     ///<    Array of tasks_entry to be executed, needs memory allocation
};

#endif /* SCHEDULER_HPP_ */