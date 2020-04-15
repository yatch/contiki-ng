/*
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
 * \addtogroup link-layer
 * @{
 */
/**
 * \defgroup msf 6TiSCH Minimal Scheduling Function (MSF)
 * @{
 */

/**
 * \file
 *         MSF shell API
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 *         alexrayne <alexraynepe196@gmail.com>
 */

#ifndef _MSF_SHELL_H
#define _MSF_SHELL_H

#include <stdint.h>

#include "services/shell/shell.h"

/**
 * \brief Handle a given sub-command of MSF
 * \param output A pointer to shell_output_func
 * \param args A pointer to user inputs
 */
void msf_shell_sub_cmd(shell_output_func output, char *args);


/**
 * \brief Show scheduled negotiated cells in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_negotiated_cells(shell_output_func output);

/**
 * \brief Show scheduled automated cells in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_autonomous_cells(shell_output_func output);

/**
 * \brief Show MSF counters in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_counters(shell_output_func output);

#endif /* !_MSF_H */
/** @} */
/** @} */
