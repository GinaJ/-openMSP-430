/*===========================================================================*/
/* Copyright (C) 2001 Authors                                                */
/*                                                                           */
/* This source file may be used and distributed without restriction provided */
/* that this copyright statement is not removed from the file and that any   */
/* derivative work contains the original copyright notice and the associated */
/* disclaimer.                                                               */
/*                                                                           */
/* This source file is free software; you can redistribute it and/or modify  */
/* it under the terms of the GNU Lesser General Public License as published  */
/* by the Free Software Foundation; either version 2.1 of the License, or    */
/* (at your option) any later version.                                       */
/*                                                                           */
/* This source is distributed in the hope that it will be useful, but WITHOUT*/
/* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or     */
/* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public       */
/* License for more details.                                                 */
/*                                                                           */
/* You should have received a copy of the GNU Lesser General Public License  */
/* along with this source; if not, write to the Free Software Foundation,    */
/* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA        */
/*                                                                           */
/*===========================================================================*/
/*                   TWO-OPERAND ARITHMETIC: MOV INSTRUCTION                 */
/*---------------------------------------------------------------------------*/
/* Test the MOV instruction with all addressing modes                        */
/*                                                                           */
/* Author(s):                                                                */
/*             - Olivier Girard,    olgirard@gmail.com                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* $Rev: 134 $                                                                */
/* $LastChangedBy: olivier.girard $                                          */
/* $LastChangedDate: 2012-03-22 21:31:06 +0100 (Thu, 22 Mar 2012) $          */
/*===========================================================================*/

initial
   begin
      $display(" ===============================================");
      $display("|                 START SIMULATION              |");
      $display(" ===============================================");
      repeat(5) @(posedge mclk);
      stimulus_done = 0;

      // Check reset values
      //--------------------------------------------------------
      if (r2 !==16'h0000) tb_error("R2  reset value");
      if (r3 !==16'h0000) tb_error("R3  reset value");
      if (r4 !==16'h0000) tb_error("R4  reset value");
      if (r5 !==16'h0000) tb_error("R5  reset value");
      if (r6 !==16'h0000) tb_error("R6  reset value");
      if (r7 !==16'h0000) tb_error("R7  reset value");
      if (r8 !==16'h0000) tb_error("R8  reset value");
      if (r9 !==16'h0000) tb_error("R9  reset value");
      if (r10!==16'h0000) tb_error("R10 reset value");
      if (r11!==16'h0000) tb_error("R11 reset value");
      if (r12!==16'h0000) tb_error("R12 reset value");
      if (r13!==16'h0000) tb_error("R13 reset value");
      if (r14!==16'h0000) tb_error("R14 reset value");
      if (r15!==16'h0000) tb_error("R15 reset value");
      
      @(posedge mclk);

      stimulus_done = 1;
   end

