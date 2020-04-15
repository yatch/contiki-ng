/*
 * msf-shell.c
 *
 *      Author: alexrayne <apexraynepe196@gmail.com>
 */

/*
 * Created on: 15/04/2020
 * Copyright (c) 2019, Inria.
 * Copyright (c) 2020, alexrayne <apexraynepe196@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *         MSF external APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 *         alexrayne <apexraynepe196@gmail.com>
 */

#include <stdbool.h>
#include <string.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"
#include "net/mac/mac.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"
#include "services/shell/serial-shell.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-reserved-cell.h"
#include "msf-sixp.h"

#include "msf-shell.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

typedef void sub_cmd(shell_output_func output);

/* static functions */
static void show_help(shell_output_func output);
static void show_constants(shell_output_func output);
static void show_status(shell_output_func output);

/* variables */
static bool activated = false;
static const struct {
  const char *name;
  sub_cmd *func;
  const char *help_msg;
} sub_cmd_list[] = {
  { "help", show_help, "show this message" },
  { "cnst", show_constants, "show constants (configuration parameters)" },
  { "stat", show_status, "show current status" },
  { "nego", msf_housekeeping_show_negotiated_cells, "show negotiated cells" },
  { "auto", msf_housekeeping_show_autonomous_cells, "show autonomous cells" },
  { NULL, NULL},
};

/*---------------------------------------------------------------------------*/
static void
show_help(shell_output_func output)
{
  if(output == NULL) {
      /* do nothing */
  } else {
    SHELL_OUTPUT(output, "MSF commands\n");
    for(int i = 0; sub_cmd_list[i].name != NULL; i++) {
      SHELL_OUTPUT(output, "'> msf %s': %s\n",
                   sub_cmd_list[i].name, sub_cmd_list[i].help_msg);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
show_constants(shell_output_func output)
{
  if(output == NULL)
    /* do nothing */
    return;

  MSFConstantInfo constants[MSFCONSTANTS_NUM];
  msf_constants_describe(constants);

  for(int i = 0; constants[i].str != NULL; i++) {
      SHELL_OUTPUT(output, constants[i].str, constants[i].val);
  }
}
/*---------------------------------------------------------------------------*/
static void
show_status(shell_output_func output)
{
  const linkaddr_t *parent_addr;
  if(output == NULL) {
      /* do nothing */
  } else {
    SHELL_OUTPUT(output, "MSF status:\n");
    SHELL_OUTPUT(output, "o activated           : %s\n",
                 activated ? "yes" : "not yet");
    SHELL_OUTPUT(output, "o parent              : ");
    shell_output_lladdr(output,
                        (parent_addr = msf_housekeeping_get_parent_addr()));
    SHELL_OUTPUT(output, "\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_shell_sub_cmd(shell_output_func output, char *args)
{
  char *next_args;
  const char *sub_cmd;
  assert(output != NULL);

  SHELL_ARGS_INIT(args, next_args);
  SHELL_ARGS_NEXT(args, next_args);

  if(args == NULL) {
    sub_cmd = "help";
  } else {
    sub_cmd = (const char*)args;
  }

  for(int i = 0; sub_cmd_list[i].name != NULL; i++) {
    if(strcmp(sub_cmd_list[i].name, sub_cmd) == 0) {
      sub_cmd_list[i].func(output);
      return;
    }
  }
  /* if we get here, no sub-command is found */
  show_help(output);
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_show_negotiated_cells(shell_output_func output)
{
  tsch_slotframe_t *slotframe = msf_negotiated_cell_get_slotframe();
  if(output == NULL) {
    /* do nothing */
  } else {
    SHELL_OUTPUT(output, "MSF Negotiated Cells:");
    if(slotframe == NULL || list_head(slotframe->links_list) == NULL) {
      SHELL_OUTPUT(output, " (none)\n");
    } else {
      uint16_t num_tx;
      SHELL_OUTPUT(output, "\n type, sl_off, ch_off,  PDR, addr\n");
      for(tsch_link_t *cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = list_item_next(cell)) {
        SHELL_OUTPUT(output, "o ");
        SHELL_OUTPUT(output, " %s,    %3u,    %3u, ",
                     cell->link_options & LINK_OPTION_TX ? "TX" : "RX",
                     cell->timeslot,
                     cell->channel_offset);
        if(cell->link_options & LINK_OPTION_RESERVED_LINK ||
           cell->link_options & LINK_OPTION_LINK_TO_DELETE) {
          /* skip it */
          continue;
        } else if(cell->link_options & LINK_OPTION_TX &&
                  (num_tx = msf_negotiated_cell_get_num_tx(cell)) > 0) {
          SHELL_OUTPUT(output, "%3u%%, ",
                       msf_negotiated_cell_get_num_tx_ack(cell) * 100 /
                       num_tx);
        } else {
          SHELL_OUTPUT(output, " N/A, ");
        }
        shell_output_lladdr(output, &cell->addr);
        SHELL_OUTPUT(output, "\n");
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_housekeeping_show_autonomous_cells(shell_output_func output)
{
  tsch_slotframe_t *slotframe = msf_autonomous_cell_get_slotframe();

  if(output == NULL) {
    /* do nothig */
  } else {
    SHELL_OUTPUT(output, "MSF Autonomous Cells:");
    if(slotframe == NULL || list_head(slotframe->links_list) == NULL) {
      SHELL_OUTPUT(output, " (none)\n");
    } else {
      SHELL_OUTPUT(output, "\n type, slot, ch_off, addr\n");
      for(tsch_link_t *cell = list_head(slotframe->links_list);
          cell != NULL;
          cell = list_item_next(cell)) {
        SHELL_OUTPUT(output, "o ");
        SHELL_OUTPUT(output, " %s,  %3u,   %3u, ",
                     cell->link_options & LINK_OPTION_TX ? "TX" : "RX",
                     cell->timeslot,
                     cell->channel_offset);
        shell_output_lladdr(output, &cell->addr);
        SHELL_OUTPUT(output, "\n");
      }
    }
  }
}
