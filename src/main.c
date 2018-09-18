/******************************************************************************
 *
 *  $Id: main.c,v 3bdd7a747fae 2012/09/20 13:28:25 fp $
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh-essen.com>
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>

#include "ecrt.h"

RT_TASK my_task;

static int run = 1;

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;

/****************************************************************************/

// process data

#define DigOutSlave01_Pos 0, 0

#define ECATIO_16IN_16OUT 0x00000A09, 0x00201

// offsets for PDO entries
static unsigned int off_dig_out0 = 0;
static unsigned int off_dig_out1 = 0;
static unsigned int off_dig_in0 = 0;
static unsigned int off_dig_in1 = 0;

// process data

const static ec_pdo_entry_reg_t domain1_regs[] = {
   { DigOutSlave01_Pos, ECATIO_16IN_16OUT, 0x7001, 0x01, &off_dig_out0, NULL },
   { DigOutSlave01_Pos, ECATIO_16IN_16OUT, 0x7001, 0x02, &off_dig_out1, NULL },
   { DigOutSlave01_Pos, ECATIO_16IN_16OUT, 0x6001, 0x01, &off_dig_in0, NULL },
   { DigOutSlave01_Pos, ECATIO_16IN_16OUT, 0x6001, 0x02, &off_dig_in1, NULL },
   {}
};

/****************************************************************************/

/* Slave 1, "EL2004"
 * Vendor ID:       0x00000002
 * Product code:    0x07d43052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_1_pdo_entries_out[] = {
   {0x7001, 0x01, 8}, /* Output */
   {0x7001, 0x02, 8}, /* Output */
};

ec_pdo_entry_info_t slave_1_pdo_entries_in[] = {
	{ 0x6001, 0x01, 8 }, /* Input */
	{ 0x6001, 0x02, 8 }, /* Input */
};

ec_pdo_info_t slave_1_pdos[] = {
	{ 0x1600, 1, slave_1_pdo_entries_out + 0 }, /* output0-7 */
	{ 0x1601, 1, slave_1_pdo_entries_out + 1 }, /* output8-15 */
	{ 0x1a00, 1, slave_1_pdo_entries_in + 0 }, /* input0-7 */
	{ 0x1a01, 1, slave_1_pdo_entries_in + 1 }, /* input8-15 */
};

ec_sync_info_t slave_1_syncs[] = {
   { 0, EC_DIR_OUTPUT, 2, slave_1_pdos + 0, EC_WD_ENABLE },
   { 1, EC_DIR_INPUT, 2, slave_1_pdos + 2, EC_WD_ENABLE },
   {0xff}
};

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

	ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

	ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void my_task_proc(void *arg)
{
	int cycle_counter = 0;
    unsigned int blink = 0;

	rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns

	while (run) {
		rt_task_wait_period(NULL);

		cycle_counter++;

		// receive EtherCAT frames
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);

		rt_check_domain_state();

		if (!(cycle_counter % 1000)) {
			rt_check_master_state();
		}

		if (!(cycle_counter % 200)) {
			blink = !blink;
		}
		EC_READ_U8(blink ? 0x0 : domain1_pd + off_dig_in0);
		EC_READ_U8(blink ? 0x0 : domain1_pd + off_dig_in1);
		EC_WRITE_U8(domain1_pd + off_dig_out0, blink ? 0x0 : 0xFF);
		EC_WRITE_U8(domain1_pd + off_dig_out1, blink ? 0x0 : 0xFF);
		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);
	}
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    ec_slave_config_t *sc;
    int ret;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    printf("Creating slave configurations...\n");

    // Create configuration for bus coupler

    sc_dig_out_01 =
		ecrt_master_slave_config(master, DigOutSlave01_Pos, ECATIO_16IN_16OUT);
    if (!sc_dig_out_01) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return -1;
    }

    ret = rt_task_create(&my_task, "my_task", 0, 80, T_FPU);
    if (ret < 0) {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting my_task...\n");
    ret = rt_task_start(&my_task, &my_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

	while (run) {
		sched_yield();
	}

    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/****************************************************************************/
