/*
 * Copyright (c) 2019, Inria.
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
 * \addtogroup link-layer
 * @{
 */
/**
 * \defgroup msf 6TiSCH Minimal Scheduling Function (MSF)
 * @{
 */

/**
 * \file
 *         MSF external APIs and configuration
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_H
#define _MSF_H

#include <stdint.h>

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "services/shell/shell.h"

#include "msf-conf.h"

/* Constants */
/**
 * \brief Scheduling Function ID for MSF
 */
#define MSF_SFID 1

/**
 * \brief Slotframe handle of the slotframe for autonomous cells
 */
#define MSF_SLOTFRAME_HANDLE_AUTONOMOUS_CELLS 1

/**
 * \brief Slotframe handle of the slotframe for negotiated cells
 */
#define MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS 2
/**
 * \brief The maximum length of a CellList in a 6P packet
 */
#define MSF_6P_CELL_LIST_MAX_LEN 5

/* SAX parameters; not expected to be changed */
#define MSF_SAX_H0 0
#define MSF_SAX_L_BIT 0
#define MSF_SAX_R_BIT 1

/* Variables */
/**
 * \brief sixtop_sf_t of msf
 */
extern const sixtop_sf_t msf;

/* Functions */

/**
 * \brief Return whether MSF is activated or not
 * \return true if MSF is activated, otherwise false
 */
bool msf_is_activated(void);

/**
 * \brief Return if MSF schedules are ready for advertising the
 * network and applications
 * \return true if the schedules are ready, otherwise false
 */
bool msf_is_ready(void);

/**
 * \brief Activate MSF
 */
void msf_activate(void);

/**
 * \brief Deactivate MSF
 */
void msf_deactivate(void);

/**
 * \brief Handle a given sub-command of MSF
 * \param output A pointer to shell_output_func
 * \param args A pointer to user inputs
 */
void msf_shell_sub_cmd(shell_output_func output, char *args);

#endif /* !_MSF_H */
/** @} */
/** @} */
