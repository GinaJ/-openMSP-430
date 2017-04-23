//--------------------------------------------------------------------------------
// 
//                  STMicroelectronics NOIDA
// 
//                            LIBRARY GROUP
// 
//--------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------
// 
//              Verilog models for the CORE65GPSVT.CMOS065LP
//                    (CORE65GPSVT   release 5)
// 
//
//--------------------------------------------------------------------------------
// 
//Date            : Fri Jul 27 10:08:47 2007
// 
//Copyright       : STMicroelectronics N.V. 2007
//                  All rights reserved. Reproduction
//                  in whole or part is prohibited
//                  without the written consent of the
//                  copyright holder.
// 
//Address         : STMicroelectronics
//                  Plot No 2 & 3 , Institutional Area
//                  Sector 16A, Noida - 201301
//                  INDIA
//                  Central R&D / DAIS.
// 
//--------------------------------------------------------------------------------
//STMicroelectronics RESERVES THE RIGHTS TO MAKE CHANGES WITHOUT 
//NOTICE AT ANY TIME. STMicroelectronics MAKES NO WARRANTY,
//EXPRESSED, IMPLIED OR STATUTARY, INCLUDING BUT NOT LIMITED TO ANY IMPLIED
//WARRANTY OR MERCHANTABILITY OR FITNESS FOR ANY PARTICULAR PURPOSE,
//OR THAT THE USE WILL NOT INFRINGE ANY THIRD PARTY PATENT,
//COPYRIGHT OR TRADEMARK. STMicroelectronics SHALL NOT BE LIABLE 
//FOR ANY LOSS OR DAMAGE ARISING FROM THE USE OF ITS LIBRARIES OR
//SOFTWARE.
//--------------------------------------------------------------------------------

// Cell HS65_GSS1_FA1X21
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS1_FA1X21 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GSS1_FA1X21
`endcelldefine
// Cell HS65_GSS1_FA1X35
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS1_FA1X35 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GSS1_FA1X35
`endcelldefine
// Cell HS65_GSS1_FA1X9
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS1_FA1X9 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GSS1_FA1X9
`endcelldefine
// Cell HS65_GSS1_HA1X8
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS1_HA1X8 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GSS1_HA1X8
`endcelldefine
// Cell HS65_GSS2_FA1X18
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS2_FA1X18 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GSS2_FA1X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQNX18 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_DFPHQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQNX27 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_DFPHQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQNX35 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_DFPHQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQX18 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_DFPHQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQX27 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_DFPHQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPHQX35 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_DFPHQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQNX18 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQNX27 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQNX35 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQX18 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQX27 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPQX35 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQNX18 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPRQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQNX27 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPRQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQNX35 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPRQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQX18 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQX27 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPRQX35 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPRQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQNX18 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPSQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQNX27 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPSQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQNX35 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GSS_DFPSQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQX18 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPSQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQX27 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPSQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_DFPSQX35 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GSS_DFPSQX35
`endcelldefine
// Cell HS65_GSS_SDFPHQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPHQNTX18 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPHQNTX18

`endcelldefine
// Cell HS65_GSS_SDFPHQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPHQNTX27 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPHQNTX27

`endcelldefine
// Cell HS65_GSS_SDFPHQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPHQNTX35 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPHQNTX35

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQNX18 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GSS_SDFPHQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQNX27 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GSS_SDFPHQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQNX35 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GSS_SDFPHQNX35
`endcelldefine
// CELL HS65_GSS_SDFPHQTX18

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GSS_SDFPHQTX18 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GSS_SDFPHQTX18 
`endcelldefine
// CELL HS65_GSS_SDFPHQTX27

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GSS_SDFPHQTX27 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GSS_SDFPHQTX27 
`endcelldefine
// CELL HS65_GSS_SDFPHQTX35

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GSS_SDFPHQTX35 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GSS_SDFPHQTX35 
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQX18 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GSS_SDFPHQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQX27 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GSS_SDFPHQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPHQX35 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GSS_SDFPHQX35
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQNTX18
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQNTX18
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQNTX18 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQNTX18
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQNTX27
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQNTX27
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQNTX27 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQNTX27
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQNTX35
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQNTX35
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQNTX35 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQNTX35
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQTX18
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQTX18
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQTX18 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQTX18
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQTX27
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQTX27
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQTX27 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQTX27
`endcelldefine
// Verilog model view for HS65_GSS_SDFPHRQTX35
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GSS_SDFPHRQTX35
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GSS_SDFPHRQTX35 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GSS_SDFPHRQTX35
`endcelldefine
// Cell HS65_GSS_SDFPQNTX18

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GSS_SDFPQNTX18 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GSS_SDFPQNTX18
`endcelldefine
// Cell HS65_GSS_SDFPQNTX27

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GSS_SDFPQNTX27 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GSS_SDFPQNTX27
`endcelldefine
// Cell HS65_GSS_SDFPQNTX35

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GSS_SDFPQNTX35 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GSS_SDFPQNTX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQNX18 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQNX27 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQNX35 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPQNX35
`endcelldefine
// CELL HS65_GSS_SDFPQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPQTX18 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPQTX18
`endcelldefine
// CELL HS65_GSS_SDFPQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPQTX27 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPQTX27
`endcelldefine
// CELL HS65_GSS_SDFPQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPQTX35 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPQTX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQX18 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQX27 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPQX35 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPQX35
`endcelldefine
// Cell HS65_GSS_SDFPRQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPRQNTX18 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GSS_SDFPRQNTX18

`endcelldefine
// Cell HS65_GSS_SDFPRQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPRQNTX27 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GSS_SDFPRQNTX27

`endcelldefine
// Cell HS65_GSS_SDFPRQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPRQNTX35 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GSS_SDFPRQNTX35

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQNX18 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPRQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQNX27 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPRQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQNX35 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPRQNX35
`endcelldefine
// CELL HS65_GSS_SDFPRQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPRQTX18 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPRQTX18
`endcelldefine
// CELL HS65_GSS_SDFPRQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPRQTX27 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPRQTX27
`endcelldefine
// CELL HS65_GSS_SDFPRQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPRQTX35 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GSS_SDFPRQTX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQX18 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQX27 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPRQX35 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPRQX35
`endcelldefine
// Cell HS65_GSS_SDFPSQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPSQNTX18 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPSQNTX18

`endcelldefine
// Cell HS65_GSS_SDFPSQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPSQNTX27 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPSQNTX27

`endcelldefine
// Cell HS65_GSS_SDFPSQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GSS_SDFPSQNTX35 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GSS_SDFPSQNTX35

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQNX18 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPSQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQNX27 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPSQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQNX35 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GSS_SDFPSQNX35
`endcelldefine
// CELL HS65_GSS_SDFPSQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPSQTX18 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GSS_SDFPSQTX18
`endcelldefine
// CELL HS65_GSS_SDFPSQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPSQTX27 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GSS_SDFPSQTX27
`endcelldefine
// CELL HS65_GSS_SDFPSQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GSS_SDFPSQTX35 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GSS_SDFPSQTX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQX18 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPSQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQX27 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPSQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_SDFPSQX35 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GSS_SDFPSQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR2X12 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR2X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR2X18 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR2X24 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR2X24
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR2X3 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR2X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR2X6 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR2X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR3X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR3X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR3X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR3X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XNOR3X8 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GSS_XNOR3X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR2X12 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GSS_XOR2X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR2X17 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GSS_XOR2X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR2X23 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GSS_XOR2X23
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR2X3 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GSS_XOR2X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR2X6 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GSS_XOR2X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR3X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GSS_XOR3X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR3X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GSS_XOR3X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GSS_XOR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GSS_XOR3X8 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GSS_XOR3X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2ABX18 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, A) ;
	not    U2 (INTERNAL2, B) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AND2ABX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2ABX27 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, A) ;
	not    U2 (INTERNAL2, B) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AND2ABX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2ABX35 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, A) ;
	not    U2 (INTERNAL2, B) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AND2ABX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2ABX9 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, A) ;
	not    U2 (INTERNAL2, B) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AND2ABX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2X18 (Z, A, B);

	output Z;
	input A;
	input B;

	and   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_AND2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2X27 (Z, A, B);

	output Z;
	input A;
	input B;

	and   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_AND2X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2X35 (Z, A, B);

	output Z;
	input A;
	input B;

	and   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_AND2X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2X4 (Z, A, B);

	output Z;
	input A;
	input B;

	and   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_AND2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND2X9 (Z, A, B);

	output Z;
	input A;
	input B;

	and   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_AND2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3ABCX18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_AND3ABCX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3ABCX27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_AND3ABCX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3ABCX35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_AND3ABCX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3ABCX9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_AND3ABCX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_AND3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_AND3X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_AND3X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_AND3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_AND3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND4X13 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_AND4X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND4X19 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_AND4X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND4X25 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_AND4X25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND4X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_AND4X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AND4X6 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_AND4X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO112X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_AO112X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO112X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_AO112X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO112X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_AO112X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO112X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_AO112X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO112X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_AO112X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO12X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_AO12X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO12X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_AO12X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO12X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_AO12X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO12X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_AO12X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO12X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B) ;
	or   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_AO12X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO212X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_AO212X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO212X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_AO212X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO212X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_AO212X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO212X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_AO212X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO212X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_AO212X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO222X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	and    U3 (INTERNAL3, E, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2, INTERNAL3) ;



endmodule // HS65_GS_AO222X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO222X27 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	and    U3 (INTERNAL3, E, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2, INTERNAL3) ;



endmodule // HS65_GS_AO222X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO222X35 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	and    U3 (INTERNAL3, E, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2, INTERNAL3) ;



endmodule // HS65_GS_AO222X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO222X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	and    U3 (INTERNAL3, E, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2, INTERNAL3) ;



endmodule // HS65_GS_AO222X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO222X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	and    U3 (INTERNAL3, E, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2, INTERNAL3) ;



endmodule // HS65_GS_AO222X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO22X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO22X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO22X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO22X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO22X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO22X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO22X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO22X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO22X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B) ;
	and    U2 (INTERNAL2, C, D) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO22X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO311X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_AO311X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO311X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_AO311X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO311X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_AO311X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO311X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_AO311X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO311X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_AO311X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO312X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL2, INTERNAL3, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO312X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO312X27 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL2, INTERNAL3, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO312X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO312X35 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL2, INTERNAL3, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO312X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO312X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL2, INTERNAL3, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO312X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO312X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL2, INTERNAL3, F) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO312X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO31X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_AO31X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO31X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_AO31X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO31X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_AO31X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO31X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_AO31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO31X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C) ;
	or   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_AO31X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO32X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO32X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO32X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO32X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO32X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO32X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO32X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO32X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO32X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO32X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO33X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E, F) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO33X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO33X27 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E, F) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO33X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO33X35 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E, F) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO33X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO33X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E, F) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO33X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AO33X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL1, A, B, C) ;
	and    U2 (INTERNAL2, D, E, F) ;
	or   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_AO33X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X13 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X26 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X26
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X7 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI112X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI112X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X17 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X23 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X23
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X46 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X46
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI12X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI12X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X10 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X15 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X20 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X31 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X31
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X41 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X41
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI13X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI13X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X13 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X26 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X26
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X7 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI211X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI211X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI212X13 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI212X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI212X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI212X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI212X2 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI212X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI212X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI212X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI212X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI212X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X17 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X23 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X23
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X46 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X46
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI21X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI21X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI222X13 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL4, E, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI222X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI222X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL4, E, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI222X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI222X2 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL4, E, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI222X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI222X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL4, E, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI222X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI222X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL4, E, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI222X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X11 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X17 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X22 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X33 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X44 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X6 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI22X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI22X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI311X12 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI311X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI311X16 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI311X16
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI311X2 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI311X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI311X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI311X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI311X8 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI311X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI312X13 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL4, D, E) ;
	or    U3 (INTERNAL3, INTERNAL4, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI312X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI312X17 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL4, D, E) ;
	or    U3 (INTERNAL3, INTERNAL4, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI312X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI312X2 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL4, D, E) ;
	or    U3 (INTERNAL3, INTERNAL4, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI312X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI312X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL4, D, E) ;
	or    U3 (INTERNAL3, INTERNAL4, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI312X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI312X8 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL4, D, E) ;
	or    U3 (INTERNAL3, INTERNAL4, F) ;
	or    U4 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI312X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X10 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X15 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X20 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X31 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X31
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X41 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X41
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI31X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI31X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI32X10 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI32X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI32X15 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI32X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI32X20 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI32X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI32X3 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI32X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI32X5 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI32X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI33X14 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E, F) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI33X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI33X19 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E, F) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI33X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI33X2 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E, F) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI33X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI33X5 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E, F) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI33X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_AOI33X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	and    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL3, D, E, F) ;
	or    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_AOI33X9
`endcelldefine
// CELL HS65_GS_BDECNX20 

 `celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_BDECNX20 (Z, DN, DNM1, NDBL, ZNN, ZNP);

   output Z;
   input DN;
   input DNM1;
   input NDBL;
   input ZNN;
   input ZNP;


   not  u0 (DNX, DN);
   and  u1 (AndNDBLDNXZNN_, NDBL, DNX, ZNN);
   not  u2 (NDBLX, NDBL);
   not  u3 (DNM1X, DNM1);
   and  u4 (AndNDBLXDNM1XZNN_, NDBLX, DNM1X, ZNN);
   and  u5 (AndNDBLXDNM1ZNP_, NDBLX, DNM1, ZNP);
   and  u6 (AndNDBLDNZNP_, NDBL, DN, ZNP);
   nor #1 u7 (Z, AndNDBLDNXZNN_, AndNDBLXDNM1XZNN_, AndNDBLXDNM1ZNP_, AndNDBLDNZNP_);




endmodule // HS65_GS_BDECNX20

 `endcelldefine
// END : HS65_GS_BDECNX20 
// CELL HS65_GS_BDECNX4 

 `celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_BDECNX4 (Z, DN, DNM1, NDBL, ZNN, ZNP);

   output Z;
   input DN;
   input DNM1;
   input NDBL;
   input ZNN;
   input ZNP;


   not  u0 (DNX, DN);
   and  u1 (AndNDBLDNXZNN_, NDBL, DNX, ZNN);
   not  u2 (NDBLX, NDBL);
   not  u3 (DNM1X, DNM1);
   and  u4 (AndNDBLXDNM1XZNN_, NDBLX, DNM1X, ZNN);
   and  u5 (AndNDBLXDNM1ZNP_, NDBLX, DNM1, ZNP);
   and  u6 (AndNDBLDNZNP_, NDBL, DN, ZNP);
   nor #1 u7 (Z, AndNDBLDNXZNN_, AndNDBLXDNM1XZNN_, AndNDBLXDNM1ZNP_, AndNDBLDNZNP_);




endmodule // HS65_GS_BDECNX4

 `endcelldefine
// END : HS65_GS_BDECNX4 
// CELL HS65_GS_BDECNX9 

 `celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_BDECNX9 (Z, DN, DNM1, NDBL, ZNN, ZNP);

   output Z;
   input DN;
   input DNM1;
   input NDBL;
   input ZNN;
   input ZNP;


   not  u0 (DNX, DN);
   and  u1 (AndNDBLDNXZNN_, NDBL, DNX, ZNN);
   not  u2 (NDBLX, NDBL);
   not  u3 (DNM1X, DNM1);
   and  u4 (AndNDBLXDNM1XZNN_, NDBLX, DNM1X, ZNN);
   and  u5 (AndNDBLXDNM1ZNP_, NDBLX, DNM1, ZNP);
   and  u6 (AndNDBLDNZNP_, NDBL, DN, ZNP);
   nor #1 u7 (Z, AndNDBLDNXZNN_, AndNDBLXDNM1XZNN_, AndNDBLXDNM1ZNP_, AndNDBLDNZNP_);




endmodule // HS65_GS_BDECNX9

 `endcelldefine
// END : HS65_GS_BDECNX9 
// CELL HS65_GS_BENCX41

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_BENCX41 (DBL, NZP, NZN, Y0, Y1, Y2);

	output DBL;
	output NZP;
	output NZN;
	input Y0;
	input Y1;
	input Y2;

	xor    U1 (INTERNAL1, Y0, Y1) ;
	not   #1 U2 (DBL, INTERNAL1) ;
	and    U3 (INTERNAL4, Y0, Y1) ;
	not    U4 (INTERNAL3, INTERNAL4) ;
	and    U5 (INTERNAL2, Y2, INTERNAL3) ;
	not   #1 U6 (NZP, INTERNAL2) ;
	not    U7 (INTERNAL6, Y2) ;
	not    U8 (INTERNAL9, Y0) ;
	not    U9 (INTERNAL10, Y1) ;
	and    U10 (INTERNAL8, INTERNAL9, INTERNAL10) ;
	not    U11 (INTERNAL7, INTERNAL8) ;
	and    U12 (INTERNAL5, INTERNAL6, INTERNAL7) ;
	not   #1 U13 (NZN, INTERNAL5) ;



endmodule // HS65_GS_BENCX41

`endcelldefine
// CELL HS65_GS_BENCX71

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_BENCX71 (DBL, NZP, NZN, Y0, Y1, Y2);

	output DBL;
	output NZP;
	output NZN;
	input Y0;
	input Y1;
	input Y2;

	xor    U1 (INTERNAL1, Y0, Y1) ;
	not   #1 U2 (DBL, INTERNAL1) ;
	and    U3 (INTERNAL4, Y0, Y1) ;
	not    U4 (INTERNAL3, INTERNAL4) ;
	and    U5 (INTERNAL2, Y2, INTERNAL3) ;
	not   #1 U6 (NZP, INTERNAL2) ;
	not    U7 (INTERNAL6, Y2) ;
	not    U8 (INTERNAL9, Y0) ;
	not    U9 (INTERNAL10, Y1) ;
	and    U10 (INTERNAL8, INTERNAL9, INTERNAL10) ;
	not    U11 (INTERNAL7, INTERNAL8) ;
	and    U12 (INTERNAL5, INTERNAL6, INTERNAL7) ;
	not   #1 U13 (NZN, INTERNAL5) ;



endmodule // HS65_GS_BENCX71

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX106 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX106
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX13 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX142 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX142
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX18 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX2 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX213 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX213
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX22 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX27 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX284 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX284
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX31 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX31
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX35 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX4 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX40 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX40
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX44 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX49 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX49
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX53 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX53
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX62 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX62
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX7 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX71 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX71
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BFX9 (Z, A);

	output Z;
	input A;

	buf   #1 U1 (Z, A) ;



endmodule // HS65_GS_BFX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BK1SX1 (TE, BB);

	input TE;
	inout BB;

	nor    U1 (IBB, TE, BB) ;
	not  (weak0,weak1) #1 U2 (BB, IBB) ;
endmodule // HS65_GS_BK1SX1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
	module HS65_GS_BK1X1 (BB);

        inout BB;

        not    U1 (IBB, BB) ;
        not  (weak0,weak1) #1 U2 (BB, IBB) ;
endmodule // HS65_LS_BK1SX1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BTHX10 (Z, A, E);

	output Z;
	input E;
	input A;

	bufif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_BTHX10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BTHX15 (Z, A, E);

	output Z;
	input E;
	input A;

	bufif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_BTHX15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BTHX20 (Z, A, E);

	output Z;
	input E;
	input A;

	bufif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_BTHX20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BTHX25 (Z, A, E);

	output Z;
	input E;
	input A;

	bufif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_BTHX25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_BTHX30 (Z, A, E);

	output Z;
	input E;
	input A;

	bufif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_BTHX30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I1X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	and   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I1X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I1X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	and   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I1X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I1X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	and   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I1X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I1X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	and   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I1X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I1X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL1, INTERNAL2, C) ;
	and   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I1X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I6X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	or   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I6X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I6X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	or   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I6X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I6X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	or   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I6X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I6X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	or   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I6X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CB4I6X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	or   #1 U3 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_CB4I6X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I1X11 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, C) ;
	and    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I1X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I1X16 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, C) ;
	and    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I1X16
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I1X21 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, C) ;
	and    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I1X21
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I1X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, C) ;
	and    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I1X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I1X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, C) ;
	and    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I1X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I6X14 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL3, A, B) ;
	and    U2 (INTERNAL2, INTERNAL3, C) ;
	or    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I6X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I6X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL3, A, B) ;
	and    U2 (INTERNAL2, INTERNAL3, C) ;
	or    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I6X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I6X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL3, A, B) ;
	and    U2 (INTERNAL2, INTERNAL3, C) ;
	or    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I6X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I6X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL3, A, B) ;
	and    U2 (INTERNAL2, INTERNAL3, C) ;
	or    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I6X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_CBI4I6X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL3, A, B) ;
	and    U2 (INTERNAL2, INTERNAL3, C) ;
	or    U3 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_CBI4I6X9
`endcelldefine
// CELL HS65_GS_D42COX9

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_D42COX9 (CO, S, C, I4, I3, I2, I1, CI);

	output S;
	output C;
	output CO;
	input CI;
	input I1;
	input I2;
	input I3;
	input I4;

	xor    u0 (HS65_GS_D42COX9INTER0, I1, I2, I3, I4, CI) ;
	not   #1 u1 (S, HS65_GS_D42COX9INTER0) ;
	xor    u2 (IN1, I1, I2, I3, I4) ;
	and    u3 (IN2, CI, IN1) ;
	or    u4 (HS65_GS_D42COX9INTER1, IN1, I4) ;
	not    u5 (IN3, HS65_GS_D42COX9INTER1) ;
	or    u6 (HS65_GS_D42COX9INTER2, IN2, IN3) ;
	not   #1 u7 (C, HS65_GS_D42COX9INTER2) ;
	and    u8 (IN4, I1, I2) ;
	xor    u9 (HS65_GS_D42COX9INTER3, I1, I2) ;
	and    u10 (IN5, HS65_GS_D42COX9INTER3, I3) ;
	or    u11 (HS65_GS_D42COX9INTER4, IN4, IN5) ;
	not   #1 u12 (CO, HS65_GS_D42COX9INTER4) ;



endmodule // HS65_GS_D42COX9

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQNX18 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_DFPHQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQNX27 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_DFPHQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQNX35 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_DFPHQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQNX4 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_DFPHQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQNX9 (QN, D, E, CP);

	output QN;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_DFPHQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQX18 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_DFPHQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQX27 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_DFPHQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQX35 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_DFPHQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQX4 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_DFPHQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPHQX9 (Q, D, E, CP);

	output Q;
	input CP;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, D1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_DFPHQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQNX18 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQNX27 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQNX35 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQNX4 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQNX9 (QN, D, CP);

	output QN;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQX18 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQX27 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQX35 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQX4 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPQX9 (Q, D, CP);

	output Q;
	input D;
	input CP;

	CORE65GPSVT_U_FD_P   U1 (IQ, D, CP ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQNX18 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPRQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQNX27 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPRQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQNX35 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPRQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQNX4 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPRQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQNX9 (QN, D, CP, RN);

	output QN;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPRQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQX18 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQX27 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQX35 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPRQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQX4 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPRQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPRQX9 (Q, D, CP, RN);

	output Q;
	input D;
	input CP;
	input RN;

	CORE65GPSVT_U_FD_P_RN   U1 (IQ, D, CP, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPRQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQNX18 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPSQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQNX27 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPSQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQNX35 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPSQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQNX4 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPSQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQNX9 (QN, D, CP, SN);

	output QN;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_DFPSQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQX18 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPSQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQX27 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPSQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQX35 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPSQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQX4 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPSQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_DFPSQX9 (Q, D, CP, SN);

	output Q;
	input D;
	input CP;
	input SN;

	CORE65GPSVT_U_FD_P_SN   U1 (IQ, D, CP, SN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_DFPSQX9
`endcelldefine
// Cell HS65_GS_FA1X18
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_FA1X18 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_FA1X18
`endcelldefine
// Cell HS65_GS_FA1X27
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_FA1X27 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_FA1X27
`endcelldefine
// Cell HS65_GS_FA1X35
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_FA1X35 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_FA1X35
`endcelldefine
// Cell HS65_GS_FA1X4
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_FA1X4 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_FA1X4
`endcelldefine
// Cell HS65_GS_FA1X9
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_FA1X9 (S0, CO, A0, B0, CI);

        output S0;
        output CO;
        input A0;
        input B0;
        input CI;

        xor   #1 U1 (S0, A0, B0, CI) ;
        or    U2 (INTERNAL2, A0, B0) ;
        and    U3 (INTERNAL1, INTERNAL2, CI) ;
        and    U4 (INTERNAL3, A0, B0) ;
        or   #1 U5 (CO, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_FA1X9
`endcelldefine
// Cell HS65_GS_HA1X18
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_HA1X18 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GS_HA1X18
`endcelldefine
// Cell HS65_GS_HA1X27
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_HA1X27 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GS_HA1X27
`endcelldefine
// Cell HS65_GS_HA1X35
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_HA1X35 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GS_HA1X35
`endcelldefine
// Cell HS65_GS_HA1X4
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_HA1X4 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GS_HA1X4
`endcelldefine
// Cell HS65_GS_HA1X9
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_HA1X9 (S0, CO, A0, B0);

        output S0;
        output CO;
        input A0;
        input B0;

        xor   #1 U1 (S0, A0, B0) ;
        and   #1 U2 (CO, A0, B0) ;



endmodule // HS65_GS_HA1X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_ITHX10 (Z, A, E);

	output Z;
	input A;
	input E;

	notif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_ITHX10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_ITHX15 (Z, A, E);

	output Z;
	input A;
	input E;

	notif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_ITHX15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_ITHX20 (Z, A, E);

	output Z;
	input A;
	input E;

	notif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_ITHX20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_ITHX25 (Z, A, E);

	output Z;
	input A;
	input E;

	notif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_ITHX25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_ITHX30 (Z, A, E);

	output Z;
	input A;
	input E;

	notif1   #1 U1 (Z, A, E) ;



endmodule // HS65_GS_ITHX30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX106 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX106
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX13 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX142 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX142
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX18 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX2 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX213 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX213
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX22 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX27 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX284 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX284
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX31 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX31
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX35 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX4 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX40 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX40
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX44 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX49 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX49
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX53 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX53
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX62 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX62
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX7 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX71 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX71
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_IVX9 (Z, A);

	output Z;
	input A;

	not   #1 U1 (Z, A) ;



endmodule // HS65_GS_IVX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQNX18 (QN, D, G);

	output QN;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_LDHQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQNX27 (QN, D, G);

	output QN;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_LDHQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQNX35 (QN, D, G);

	output QN;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_LDHQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQNX4 (QN, D, G);

	output QN;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_LDHQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQNX9 (QN, D, G);

	output QN;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	not   #1 U2 (QN, IQ) ;
endmodule // HS65_GS_LDHQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQX18 (Q, D, G);

	output Q;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQX27 (Q, D, G);

	output Q;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQX35 (Q, D, G);

	output Q;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQX4 (Q, D, G);

	output Q;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHQX9 (Q, D, G);

	output Q;
	input D;
	input G;

	CORE65GPSVT_U_LD_P   U1 (IQ, D, G ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHRQX18 (Q, D, G, RN);

	output Q;
	input D;
	input G;
	input RN;

	CORE65GPSVT_U_LD_P_RN   U1 (IQ, D, G, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHRQX27 (Q, D, G, RN);

	output Q;
	input D;
	input G;
	input RN;

	CORE65GPSVT_U_LD_P_RN   U1 (IQ, D, G, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHRQX35 (Q, D, G, RN);

	output Q;
	input D;
	input G;
	input RN;

	CORE65GPSVT_U_LD_P_RN   U1 (IQ, D, G, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHRQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHRQX4 (Q, D, G, RN);

	output Q;
	input D;
	input G;
	input RN;

	CORE65GPSVT_U_LD_P_RN   U1 (IQ, D, G, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHRQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDHRQX9 (Q, D, G, RN);

	output Q;
	input D;
	input G;
	input RN;

	CORE65GPSVT_U_LD_P_RN   U1 (IQ, D, G, RN ) ;
	buf   #1 U2 (Q, IQ) ;
endmodule // HS65_GS_LDHRQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQNX18 (QN, D, GN);

	output QN;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_LDLQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQNX27 (QN, D, GN);

	output QN;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_LDLQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQNX35 (QN, D, GN);

	output QN;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_LDLQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQNX4 (QN, D, GN);

	output QN;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_LDLQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQNX9 (QN, D, GN);

	output QN;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_LDLQNX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQX18 (Q, D, GN);

	output Q;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQX27 (Q, D, GN);

	output Q;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQX35 (Q, D, GN);

	output Q;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQX4 (Q, D, GN);

	output Q;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLQX9 (Q, D, GN);

	output Q;
	input D;
	input GN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P   U2 (IQ, D, INTERNAL1 ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLRQX18 (Q, D, GN, RN);

	output Q;
	input D;
	input GN;
	input RN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P_RN   U2 (IQ, D, INTERNAL1, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLRQX27 (Q, D, GN, RN);

	output Q;
	input D;
	input GN;
	input RN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P_RN   U2 (IQ, D, INTERNAL1, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLRQX35 (Q, D, GN, RN);

	output Q;
	input D;
	input GN;
	input RN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P_RN   U2 (IQ, D, INTERNAL1, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLRQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLRQX4 (Q, D, GN, RN);

	output Q;
	input D;
	input GN;
	input RN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P_RN   U2 (IQ, D, INTERNAL1, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLRQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_LDLRQX9 (Q, D, GN, RN);

	output Q;
	input D;
	input GN;
	input RN;

	not    U1 (INTERNAL1, GN) ;
	CORE65GPSVT_U_LD_P_RN   U2 (IQ, D, INTERNAL1, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_LDLRQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X12 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X18 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X24 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X24
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X3 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X30 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X36 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X36
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X42 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X42
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X48 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X48
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21I1X6 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	not    U1 (INTERNAL1, D0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D1, S0) ;



endmodule // HS65_GS_MUX21I1X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X18 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X27 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X35 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X4 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X44 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X53 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X53
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X62 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X62
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X71 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X71
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX21X9 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2  #1 U1 (Z, D0, D1, S0) ;



endmodule // HS65_GS_MUX21X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX31X18 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D2, S1) ;



endmodule // HS65_GS_MUX31X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX31X27 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D2, S1) ;



endmodule // HS65_GS_MUX31X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX31X35 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D2, S1) ;



endmodule // HS65_GS_MUX31X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX31X4 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D2, S1) ;



endmodule // HS65_GS_MUX31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX31X9 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2  #1 U2 (Z, INTERNAL1, D2, S1) ;



endmodule // HS65_GS_MUX31X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX41X18 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Z, INTERNAL1, INTERNAL2, S1) ;



endmodule // HS65_GS_MUX41X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX41X27 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Z, INTERNAL1, INTERNAL2, S1) ;



endmodule // HS65_GS_MUX41X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX41X35 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Z, INTERNAL1, INTERNAL2, S1) ;



endmodule // HS65_GS_MUX41X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX41X4 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Z, INTERNAL1, INTERNAL2, S1) ;



endmodule // HS65_GS_MUX41X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUX41X9 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Z, INTERNAL1, INTERNAL2, S1) ;



endmodule // HS65_GS_MUX41X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X10 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X15 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X2 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X20 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X30 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X40 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X40
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X5 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X50 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X50
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI21X60 (Z, D0, D1, S0);

	output Z;
	input D0;
	input D1;
	input S0;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_MUXI21X60
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI31X11 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (D6, INTERNAL1, D2, S1) ;
	not   #1 U3 (Z, D6) ;



endmodule // HS65_GS_MUXI31X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI31X15 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (D6, INTERNAL1, D2, S1) ;
	not   #1 U3 (Z, D6) ;



endmodule // HS65_GS_MUXI31X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI31X2 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (D6, INTERNAL1, D2, S1) ;
	not   #1 U3 (Z, D6) ;



endmodule // HS65_GS_MUXI31X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI31X4 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (D6, INTERNAL1, D2, S1) ;
	not   #1 U3 (Z, D6) ;



endmodule // HS65_GS_MUXI31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI31X7 (Z, D0, D1, D2, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (D6, INTERNAL1, D2, S1) ;
	not   #1 U3 (Z, D6) ;



endmodule // HS65_GS_MUXI31X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI41X11 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2   U3 (NET1, INTERNAL1, INTERNAL2, S1) ;
	not   #1 U4 (Z, NET1) ;



endmodule // HS65_GS_MUXI41X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI41X15 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2   U3 (NET1, INTERNAL1, INTERNAL2, S1) ;
	not   #1 U4 (Z, NET1) ;



endmodule // HS65_GS_MUXI41X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI41X2 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2   U3 (NET1, INTERNAL1, INTERNAL2, S1) ;
	not   #1 U4 (Z, NET1) ;



endmodule // HS65_GS_MUXI41X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI41X4 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2   U3 (NET1, INTERNAL1, INTERNAL2, S1) ;
	not   #1 U4 (Z, NET1) ;



endmodule // HS65_GS_MUXI41X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MUXI41X7 (Z, D0, D1, D2, D3, S0, S1);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D0, D1, S0) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL2, D2, D3, S0) ;
	CORE65GPSVT_U_MUX2   U3 (NET1, INTERNAL1, INTERNAL2, S1) ;
	not   #1 U4 (Z, NET1) ;



endmodule // HS65_GS_MUXI41X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MX41X14 (Z, D0, D1, D2, D3, S0, S1, S2, S3);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;
	input S2;
	input S3;

	and    U1 (INTERNAL1, D0, S0) ;
	and    U2 (INTERNAL2, D1, S1) ;
	and    U3 (INTERNAL3, D2, S2) ;
	and    U4 (INTERNAL4, D3, S3) ;
	or   #1 U5 (Z, INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;



endmodule // HS65_GS_MX41X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MX41X21 (Z, D0, D1, D2, D3, S0, S1, S2, S3);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;
	input S2;
	input S3;

	and    U1 (INTERNAL1, D0, S0) ;
	and    U2 (INTERNAL2, D1, S1) ;
	and    U3 (INTERNAL3, D2, S2) ;
	and    U4 (INTERNAL4, D3, S3) ;
	or   #1 U5 (Z, INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;



endmodule // HS65_GS_MX41X21
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MX41X29 (Z, D0, D1, D2, D3, S0, S1, S2, S3);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;
	input S2;
	input S3;

	and    U1 (INTERNAL1, D0, S0) ;
	and    U2 (INTERNAL2, D1, S1) ;
	and    U3 (INTERNAL3, D2, S2) ;
	and    U4 (INTERNAL4, D3, S3) ;
	or   #1 U5 (Z, INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;



endmodule // HS65_GS_MX41X29
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MX41X4 (Z, D0, D1, D2, D3, S0, S1, S2, S3);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;
	input S2;
	input S3;

	and    U1 (INTERNAL1, D0, S0) ;
	and    U2 (INTERNAL2, D1, S1) ;
	and    U3 (INTERNAL3, D2, S2) ;
	and    U4 (INTERNAL4, D3, S3) ;
	or   #1 U5 (Z, INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;



endmodule // HS65_GS_MX41X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_MX41X7 (Z, D0, D1, D2, D3, S0, S1, S2, S3);

	output Z;
	input D0;
	input D1;
	input D2;
	input D3;
	input S0;
	input S1;
	input S2;
	input S3;

	and    U1 (INTERNAL1, D0, S0) ;
	and    U2 (INTERNAL2, D1, S1) ;
	and    U3 (INTERNAL3, D2, S2) ;
	and    U4 (INTERNAL4, D3, S3) ;
	or   #1 U5 (Z, INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;



endmodule // HS65_GS_MX41X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2AX14 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	or   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NAND2AX14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2AX21 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	or   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NAND2AX21
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2AX29 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	or   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NAND2AX29
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2AX4 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	or   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NAND2AX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2AX7 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	or   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NAND2AX7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X11 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X14 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X2 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X21 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X21
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X29 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X29
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X4 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X43 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X43
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X5 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X57 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X57
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND2X7 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND2X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3AX13 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	and    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3AX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3AX19 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	and    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3AX19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3AX25 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	and    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3AX25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3AX3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	and    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3AX3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3AX6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	and    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3AX6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X13 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X19 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X25 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X38 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X38
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X5 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X50 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X50
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4ABX13 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL2, A) ;
	not    U2 (INTERNAL3, B) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, C, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4ABX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4ABX19 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL2, A) ;
	not    U2 (INTERNAL3, B) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, C, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4ABX19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4ABX25 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL2, A) ;
	not    U2 (INTERNAL3, B) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, C, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4ABX25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4ABX3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL2, A) ;
	not    U2 (INTERNAL3, B) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, C, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4ABX3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4ABX6 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL2, A) ;
	not    U2 (INTERNAL3, B) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, C, D) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4ABX6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NAND4X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	and    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NAND4X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2AX13 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	and   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NOR2AX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2AX19 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	and   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NOR2AX19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2AX25 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	and   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NOR2AX25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2AX3 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	and   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NOR2AX3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2AX6 (Z, A, B);

	output Z;
	input A;
	input B;

	not    U1 (INTERNAL1, B) ;
	and   #1 U2 (Z, A, INTERNAL1) ;



endmodule // HS65_GS_NOR2AX6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X13 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X19 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X2 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X25 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X25
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X3 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X38 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X38
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X5 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X50 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X50
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X6 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR2X9 (Z, A, B);

	output Z;
	input A;
	input B;

	or    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3AX13 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	or    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3AX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3AX18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	or    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3AX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3AX2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	or    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3AX2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3AX4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	or    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3AX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3AX9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	not    U1 (INTERNAL2, A) ;
	or    U2 (INTERNAL1, INTERNAL2, B, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3AX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X1 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X13 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X26 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X26
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X7 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4ABX13 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL1, C) ;
	not    U2 (INTERNAL2, D) ;
	and   #1 U3 (Z, A, B, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_NOR4ABX13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4ABX18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL1, C) ;
	not    U2 (INTERNAL2, D) ;
	and   #1 U3 (Z, A, B, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_NOR4ABX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4ABX2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL1, C) ;
	not    U2 (INTERNAL2, D) ;
	and   #1 U3 (Z, A, B, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_NOR4ABX2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4ABX4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL1, C) ;
	not    U2 (INTERNAL2, D) ;
	and   #1 U3 (Z, A, B, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_NOR4ABX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4ABX9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	not    U1 (INTERNAL1, C) ;
	not    U2 (INTERNAL2, D) ;
	and   #1 U3 (Z, A, B, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_NOR4ABX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR4X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR4X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR4X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR4X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_NOR4X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C, D) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_NOR4X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA112X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_OA112X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA112X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_OA112X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA112X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_OA112X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA112X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_OA112X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA112X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C, D) ;



endmodule // HS65_GS_OA112X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA12X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_OA12X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA12X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_OA12X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA12X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_OA12X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA12X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_OA12X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA12X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL1, A, B) ;
	and   #1 U2 (Z, INTERNAL1, C) ;



endmodule // HS65_GS_OA12X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA212X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_OA212X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA212X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_OA212X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA212X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_OA212X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA212X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_OA212X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA212X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2, E) ;



endmodule // HS65_GS_OA212X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA222X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not    U5 (NET1, INTERNAL1) ;
	not   #1 U6 (Z, NET1) ;



endmodule // HS65_GS_OA222X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA222X27 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not    U5 (NET1, INTERNAL1) ;
	not   #1 U6 (Z, NET1) ;



endmodule // HS65_GS_OA222X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA222X35 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not    U5 (NET1, INTERNAL1) ;
	not   #1 U6 (Z, NET1) ;



endmodule // HS65_GS_OA222X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA222X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not    U5 (NET1, INTERNAL1) ;
	not   #1 U6 (Z, NET1) ;



endmodule // HS65_GS_OA222X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA222X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not    U5 (NET1, INTERNAL1) ;
	not   #1 U6 (Z, NET1) ;



endmodule // HS65_GS_OA222X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA22X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA22X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA22X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA22X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA22X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA22X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA22X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA22X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA22X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B) ;
	or    U2 (INTERNAL2, C, D) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA22X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA311X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_OA311X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA311X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_OA311X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA311X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_OA311X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA311X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_OA311X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA311X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D, E) ;



endmodule // HS65_GS_OA311X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA31X18 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_OA31X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA31X27 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_OA31X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA31X35 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_OA31X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA31X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_OA31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA31X9 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL1, A, B, C) ;
	and   #1 U2 (Z, INTERNAL1, D) ;



endmodule // HS65_GS_OA31X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA32X18 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA32X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA32X27 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA32X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA32X35 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA32X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA32X4 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA32X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA32X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA32X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA33X18 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E, F) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA33X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA33X27 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E, F) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA33X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA33X35 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E, F) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA33X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA33X4 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E, F) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA33X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OA33X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL1, A, B, C) ;
	or    U2 (INTERNAL2, D, E, F) ;
	and   #1 U3 (Z, INTERNAL1, INTERNAL2) ;



endmodule // HS65_GS_OA33X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X11 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X16 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X16
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X22 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X33 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X44 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI112X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI112X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X24 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X24
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X37 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X37
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X49 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X49
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X5 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI12X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI12X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X10 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X15 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X20 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X30 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X40 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X40
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI13X7 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI13X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X11 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X16 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X16
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X22 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X33 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X44 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI211X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI211X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI212X10 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI212X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI212X15 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI212X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI212X20 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI212X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI212X3 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI212X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI212X5 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3, E) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI212X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X12 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X2 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X24 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X24
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X3 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X37 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X37
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X49 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X49
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X5 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X6 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI21X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or    U1 (INTERNAL2, A, B) ;
	and    U2 (INTERNAL1, INTERNAL2, C) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI21X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI222X14 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI222X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI222X19 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI222X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI222X2 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI222X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI222X5 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI222X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI222X9 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	or    U3 (INTERNAL4, E, F) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL3, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI222X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X11 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X17 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X22 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X3 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X33 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X44 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X6 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI22X8 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B) ;
	or    U2 (INTERNAL3, C, D) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI22X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI311X10 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI311X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI311X15 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI311X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI311X2 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI311X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI311X20 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI311X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI311X5 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D, E) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI311X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X1 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X10 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X15 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X15
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X2 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X20 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X20
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X30 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X30
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X40 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X40
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X5 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI31X7 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or    U1 (INTERNAL2, A, B, C) ;
	and    U2 (INTERNAL1, INTERNAL2, D) ;
	not   #1 U3 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI31X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI32X14 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI32X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI32X19 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI32X19
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI32X2 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI32X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI32X5 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI32X5
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI32X9 (Z, A, B, C, D, E);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI32X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI33X10 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E, F) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI33X10
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI33X13 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E, F) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI33X13
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI33X2 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E, F) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI33X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI33X3 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E, F) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI33X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OAI33X6 (Z, A, B, C, D, E, F);

	output Z;
	input A;
	input B;
	input C;
	input D;
	input E;
	input F;

	or    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL3, D, E, F) ;
	and    U3 (INTERNAL1, INTERNAL2, INTERNAL3) ;
	not   #1 U4 (Z, INTERNAL1) ;



endmodule // HS65_GS_OAI33X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2ABX18 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR2ABX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2ABX27 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR2ABX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2ABX35 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR2ABX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2ABX9 (Z, A, B);

	output Z;
	input A;
	input B;

	and    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR2ABX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2X18 (Z, A, B);

	output Z;
	input A;
	input B;

	or   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_OR2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2X27 (Z, A, B);

	output Z;
	input A;
	input B;

	or   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_OR2X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2X35 (Z, A, B);

	output Z;
	input A;
	input B;

	or   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_OR2X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2X4 (Z, A, B);

	output Z;
	input A;
	input B;

	or   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_OR2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR2X9 (Z, A, B);

	output Z;
	input A;
	input B;

	or   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_OR2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3ABCX18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR3ABCX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3ABCX27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR3ABCX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3ABCX35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR3ABCX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3ABCX9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	and    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_OR3ABCX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_OR3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_OR3X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3X35 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_OR3X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_OR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	or   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_OR3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR4X14 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_OR4X14
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR4X21 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_OR4X21
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR4X29 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_OR4X29
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR4X4 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_OR4X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_OR4X7 (Z, A, B, C, D);

	output Z;
	input A;
	input B;
	input C;
	input D;

	or   #1 U1 (Z, A, B, C, D) ;



endmodule // HS65_GS_OR4X7
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO2X18 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	or    U1 (INTERNAL2, B, P) ;
	and    U2 (INTERNAL1, INTERNAL2, A) ;
	and    U3 (INTERNAL3, B, P) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO2X27 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	or    U1 (INTERNAL2, B, P) ;
	and    U2 (INTERNAL1, INTERNAL2, A) ;
	and    U3 (INTERNAL3, B, P) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO2X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO2X35 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	or    U1 (INTERNAL2, B, P) ;
	and    U2 (INTERNAL1, INTERNAL2, A) ;
	and    U3 (INTERNAL3, B, P) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO2X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO2X4 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	or    U1 (INTERNAL2, B, P) ;
	and    U2 (INTERNAL1, INTERNAL2, A) ;
	and    U3 (INTERNAL3, B, P) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO2X9 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	or    U1 (INTERNAL2, B, P) ;
	and    U2 (INTERNAL1, INTERNAL2, A) ;
	and    U3 (INTERNAL3, B, P) ;
	or   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO3X18 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, P) ;
	or    U3 (INTERNAL3, A, B, C) ;
	and   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO3X27 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, P) ;
	or    U3 (INTERNAL3, A, B, C) ;
	and   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO3X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO3X35 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, P) ;
	or    U3 (INTERNAL3, A, B, C) ;
	and   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO3X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO3X4 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, P) ;
	or    U3 (INTERNAL3, A, B, C) ;
	and   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAO3X9 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL2, A, B, C) ;
	or    U2 (INTERNAL1, INTERNAL2, P) ;
	or    U3 (INTERNAL3, A, B, C) ;
	and   #1 U4 (Z, INTERNAL1, INTERNAL3) ;



endmodule // HS65_GS_PAO3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X1 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X11 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X11
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X17 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X17
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X22 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X22
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X3 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X33 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X4 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X44 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X44
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X6 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI2X8 (Z, A, B, P);

	output Z;
	input A;
	input B;
	input P;

	and    U1 (INTERNAL3, A, B) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI2X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X1 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X1
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X12 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X12
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X16 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X16
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X2 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X2
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X24 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X24
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X3 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X3
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X33 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X33
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X4 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X6 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X6
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_PAOI3X8 (Z, A, B, C, P);

	output Z;
	input A;
	input B;
	input C;
	input P;

	and    U1 (INTERNAL3, A, B, C) ;
	or    U2 (INTERNAL2, INTERNAL3, P) ;
	or    U3 (INTERNAL4, A, B, C) ;
	and    U4 (INTERNAL1, INTERNAL2, INTERNAL4) ;
	not   #1 U5 (Z, INTERNAL1) ;



endmodule // HS65_GS_PAOI3X8
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPBTQX9 (Q, D, CP, TI, TE, TR);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;
	input TR;

	CORE65GPSVT_U_MUX2   U1 (data, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (intern, data, CP ) ;
	CORE65GPSVT_U_MUX2  #1 U3 (Q, intern, data, TR) ;
endmodule // HS65_GS_SDFPBTQX9
`endcelldefine
// Cell HS65_GS_SDFPHQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPHQNTX18 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GS_SDFPHQNTX18

`endcelldefine
// Cell HS65_GS_SDFPHQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPHQNTX27 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GS_SDFPHQNTX27

`endcelldefine
// Cell HS65_GS_SDFPHQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPHQNTX35 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GS_SDFPHQNTX35

`endcelldefine
// Cell HS65_GS_SDFPHQNTX4

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPHQNTX4 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GS_SDFPHQNTX4

`endcelldefine
// Cell HS65_GS_SDFPHQNTX9

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPHQNTX9 (QN, TQ, D, E, CP, TI, TE);

	output QN;
	output TQ;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;



endmodule // HS65_GS_SDFPHQNTX9

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQNX18 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GS_SDFPHQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQNX27 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GS_SDFPHQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQNX35 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GS_SDFPHQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQNX4 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GS_SDFPHQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQNX9 (QN, D, E, CP, TI, TE);

	output QN;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	not   #1 U4 (QN, IQ) ;
endmodule // HS65_GS_SDFPHQNX9
`endcelldefine
// CELL HS65_GS_SDFPHQTX18

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GS_SDFPHQTX18 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GS_SDFPHQTX18 
`endcelldefine
// CELL HS65_GS_SDFPHQTX27

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GS_SDFPHQTX27 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GS_SDFPHQTX27 
`endcelldefine
// CELL HS65_GS_SDFPHQTX35

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GS_SDFPHQTX35 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GS_SDFPHQTX35 
`endcelldefine
// CELL HS65_GS_SDFPHQTX4

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GS_SDFPHQTX4 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GS_SDFPHQTX4 
`endcelldefine
// CELL HS65_GS_SDFPHQTX9

`celldefine
   `timescale 1ns / 1ns
   
 

 
module HS65_GS_SDFPHQTX9 (Q, TQ, D, E, CP, TI, TE);
 
   output Q;
   output TQ;
   input CP;
   input TE;
   input TI;
   input E;
   input D;
 
 
   
 
   CORE65GPSVT_U_MUX2  u0 (Mux21IQDE_, IQ, D, E);
   CORE65GPSVT_U_MUX2  u1 (Mux21Mux21IQDE_TITE_, Mux21IQDE_, TI, TE);
 
   CORE65GPSVT_U_FD_P  u2 (   // Verilog Seq UDP
      IQ, Mux21Mux21IQDE_TITE_, CP);
 
   buf #1 u3 (Q, IQ);
   buf #1 u4 (TQ, IQ);
 
 
 
 
 
endmodule // HS65_GS_SDFPHQTX9 
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQX18 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GS_SDFPHQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQX27 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GS_SDFPHQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQX35 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GS_SDFPHQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQX4 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GS_SDFPHQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPHQX9 (Q, D, E, CP, TI, TE);

	output Q;
	input CP;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P   U3 (IQ, INTERNAL1, CP ) ;
	buf   #1 U4 (Q, IQ) ;
endmodule // HS65_GS_SDFPHQX9
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQNTX18
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQNTX18
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQNTX18 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQNTX18
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQNTX27
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQNTX27
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQNTX27 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQNTX27
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQNTX35
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQNTX35
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQNTX35 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQNTX35
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQNTX4
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQNTX4
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQNTX4 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQNTX4
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQNTX9
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQNTX9
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQNTX9 (QN, TQ, D, E, CP, RN, TI, TE);

	output QN;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U4 (QN, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQNTX9
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQTX18
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQTX18
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQTX18 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQTX18
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQTX27
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQTX27
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQTX27 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQTX27
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQTX35
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQTX35
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQTX35 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQTX35
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQTX4
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQTX4
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQTX4 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQTX4
`endcelldefine
// Verilog model view for HS65_GS_SDFPHRQTX9
//(c) 1997-2005 STMicroelectronics FTM 4.7.7.beta (Jun 22 2005 10:24:55) 24-Jun-2005
// Cell HS65_GS_SDFPHRQTX9
`celldefine
   `timescale 1ns / 1ps
    
module HS65_GS_SDFPHRQTX9 (Q, TQ, D, E, CP, RN, TI, TE);

	output Q;
	output TQ;
	input CP;
	input RN;
	input TE;
	input TI;
	input E;
	input D;

	CORE65GPSVT_U_MUX2   U1 (D1, IQ, D, E) ;
	CORE65GPSVT_U_MUX2   U2 (INTERNAL1, D1, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U3 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U4 (Q, IQ) ;
	buf   #1 U5 (TQ, IQ) ;
endmodule // HS65_GS_SDFPHRQTX9
`endcelldefine
// Cell HS65_GS_SDFPQNTX18

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_SDFPQNTX18 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GS_SDFPQNTX18
`endcelldefine
// Cell HS65_GS_SDFPQNTX27

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_SDFPQNTX27 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GS_SDFPQNTX27
`endcelldefine
// Cell HS65_GS_SDFPQNTX35

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_SDFPQNTX35 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GS_SDFPQNTX35
`endcelldefine
// Cell HS65_GS_SDFPQNTX4

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_SDFPQNTX4 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GS_SDFPQNTX4
`endcelldefine
// Cell HS65_GS_SDFPQNTX9

`celldefine
   `timescale 1ns / 1ns
   


module HS65_GS_SDFPQNTX9 (QN, TQ, D, CP, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;

endmodule // HS65_GS_SDFPQNTX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQNX18 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQNX27 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQNX35 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQNX4 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQNX9 (QN, D, CP, TI, TE);

	output QN;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPQNX9
`endcelldefine
// CELL HS65_GS_SDFPQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPQTX18 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPQTX18
`endcelldefine
// CELL HS65_GS_SDFPQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPQTX27 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPQTX27
`endcelldefine
// CELL HS65_GS_SDFPQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPQTX35 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPQTX35
`endcelldefine
// CELL HS65_GS_SDFPQTX4

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPQTX4 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPQTX4
`endcelldefine
// CELL HS65_GS_SDFPQTX9

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPQTX9 (Q, TQ, D, CP, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP);

   buf #1 u2 (Q, IQ);
buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPQTX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQX18 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQX27 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQX35 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQX4 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPQX9 (Q, D, CP, TI, TE);

	output Q;
	input D;
	input CP;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P   U2 (IQ, INTERNAL1, CP ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPQX9
`endcelldefine
// Cell HS65_GS_SDFPRQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPRQNTX18 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GS_SDFPRQNTX18

`endcelldefine
// Cell HS65_GS_SDFPRQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPRQNTX27 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GS_SDFPRQNTX27

`endcelldefine
// Cell HS65_GS_SDFPRQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPRQNTX35 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GS_SDFPRQNTX35

`endcelldefine
// Cell HS65_GS_SDFPRQNTX4

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPRQNTX4 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GS_SDFPRQNTX4

`endcelldefine
// Cell HS65_GS_SDFPRQNTX9

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPRQNTX9 (QN, TQ, D, CP, RN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;


endmodule // HS65_GS_SDFPRQNTX9

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQNX18 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPRQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQNX27 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPRQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQNX35 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPRQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQNX4 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPRQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQNX9 (QN, D, CP, RN, TI, TE);

	output QN;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPRQNX9
`endcelldefine
// CELL HS65_GS_SDFPRQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPRQTX18 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPRQTX18
`endcelldefine
// CELL HS65_GS_SDFPRQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPRQTX27 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPRQTX27
`endcelldefine
// CELL HS65_GS_SDFPRQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPRQTX35 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPRQTX35
`endcelldefine
// CELL HS65_GS_SDFPRQTX4

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPRQTX4 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPRQTX4
`endcelldefine
// CELL HS65_GS_SDFPRQTX9

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPRQTX9 (Q, TQ, D, CP, RN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input RN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2 u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_RN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, RN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);





endmodule // HS65_GS_SDFPRQTX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQX18 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPRQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQX27 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPRQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQX35 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPRQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQX4 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPRQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPRQX9 (Q, D, CP, RN, TI, TE);

	output Q;
	input D;
	input CP;
	input RN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_RN   U2 (IQ, INTERNAL1, CP, RN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPRQX9
`endcelldefine
// Cell HS65_GS_SDFPSQNTX18

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPSQNTX18 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GS_SDFPSQNTX18

`endcelldefine
// Cell HS65_GS_SDFPSQNTX27

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPSQNTX27 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GS_SDFPSQNTX27

`endcelldefine
// Cell HS65_GS_SDFPSQNTX35

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPSQNTX35 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GS_SDFPSQNTX35

`endcelldefine
// Cell HS65_GS_SDFPSQNTX4

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPSQNTX4 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GS_SDFPSQNTX4

`endcelldefine
// Cell HS65_GS_SDFPSQNTX9

`celldefine
   `timescale 1ns / 1ns
   

module HS65_GS_SDFPSQNTX9 (QN, TQ, D, CP, SN, TI, TE);

	output QN;
	output TQ;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN) ;
	not   #1 U3 (QN, IQ) ;
	buf   #1 U4 (TQ, IQ) ;



endmodule // HS65_GS_SDFPSQNTX9

`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQNX18 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPSQNX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQNX27 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPSQNX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQNX35 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPSQNX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQNX4 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPSQNX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQNX9 (QN, D, CP, SN, TI, TE);

	output QN;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	not   #1 U3 (QN, IQ) ;
endmodule // HS65_GS_SDFPSQNX9
`endcelldefine
// CELL HS65_GS_SDFPSQTX18

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPSQTX18 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GS_SDFPSQTX18
`endcelldefine
// CELL HS65_GS_SDFPSQTX27

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPSQTX27 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GS_SDFPSQTX27
`endcelldefine
// CELL HS65_GS_SDFPSQTX35

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPSQTX35 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GS_SDFPSQTX35
`endcelldefine
// CELL HS65_GS_SDFPSQTX4

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPSQTX4 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GS_SDFPSQTX4
`endcelldefine
// CELL HS65_GS_SDFPSQTX9

`celldefine
   `timescale 1ns / 1ns
   
 


module HS65_GS_SDFPSQTX9 (Q, TQ, D, CP, SN, TI, TE);

   output Q;
   output TQ;
   input D;
   input CP;
   input SN;
   input TI;
   input TE;


   

   CORE65GPSVT_U_MUX2  u0 (Mux21DTITE_, D, TI, TE);

   CORE65GPSVT_U_FD_P_SN  u1 (   // Verilog Seq UDP
      IQ, Mux21DTITE_, CP, SN);

   buf #1 u2 (Q, IQ);
   buf #1 u3 (TQ, IQ);






endmodule // HS65_GS_SDFPSQTX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQX18 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPSQX18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQX27 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPSQX27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQX35 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPSQX35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQX4 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPSQX4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_SDFPSQX9 (Q, D, CP, SN, TI, TE);

	output Q;
	input D;
	input CP;
	input SN;
	input TI;
	input TE;

	CORE65GPSVT_U_MUX2   U1 (INTERNAL1, D, TI, TE) ;
	CORE65GPSVT_U_FD_P_SN   U2 (IQ, INTERNAL1, CP, SN ) ;
	buf   #1 U3 (Q, IQ) ;
endmodule // HS65_GS_SDFPSQX9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR2X18 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR2X27 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR2X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR2X35 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR2X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR2X4 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR2X9 (Z, A, B);

	output Z;
	input A;
	input B;

	xor    U1 (INTERNAL1, A, B) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR3X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR3X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR3X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XNOR3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor    U1 (INTERNAL1, A, B, C) ;
	not   #1 U2 (Z, INTERNAL1) ;



endmodule // HS65_GS_XNOR3X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR2X18 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_XOR2X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR2X27 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_XOR2X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR2X35 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_XOR2X35
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR2X4 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_XOR2X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR2X9 (Z, A, B);

	output Z;
	input A;
	input B;

	xor   #1 U1 (Z, A, B) ;



endmodule // HS65_GS_XOR2X9
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR3X18 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_XOR3X18
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR3X27 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_XOR3X27
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR3X4 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_XOR3X4
`endcelldefine
`celldefine
   `timescale 1ns / 1ns
module HS65_GS_XOR3X9 (Z, A, B, C);

	output Z;
	input A;
	input B;
	input C;

	xor   #1 U1 (Z, A, B, C) ;



endmodule // HS65_GS_XOR3X9
`endcelldefine


primitive CORE65GPSVT_U_FD_P (Q, D, CP);
        output Q;
        input  D, CP;
        reg    Q;

        table
        //D     CP     : Q : Qn+1
         0      (01)   : ? :  0 ;
         1      (01)   : ? :  1 ;
         *      B      : ? :  - ;
         ?      (?0)   : ? :  - ;
         0      (1X)   : 0 :  0 ;
         1      (1X)   : 1 :  1 ;
         0      (0X)   : 0 :  0 ;
         0      (X1)   : 0 :  0 ;
         1      (0X)   : 1 :  1 ;
         1      (X1)   : 1 :  1 ;


        endtable

endprimitive


primitive CORE65GPSVT_U_FD_P_RN (Q, D, CP, RN);
        output Q;
        input  D, CP, RN;
        reg    Q;

        table
        //D     CP      RN    : Qn  : Qn+1
         ?      ?       0     :   ? :  0 ;
         *      ?       0     :   ? :  0 ;
         ?      *       0     :   ? :  0 ;
         1      (01)    1     :   ? :  1 ;
         0      (01)    1     :   ? :  0 ;
         *      B       1     :   ? :  - ;
         ?      (?0)    1     :   ? :  - ;
         0      (1X)    ?     :   0 :  0 ;
         1      (1X)    1     :   1 :  1 ;
         ?      B       (?1)  :   ? :  - ;
         0      X       (?1)  :   0 :  0 ;
         0      (0X)    ?     :   0 :  0 ;
         0      (X1)    ?     :   0 :  0 ;
         1      (0X)    1     :   1 :  1 ;
         1      (X1)    1     :   1 :  1 ;
         0      (01)    X     :   ? :  0 ;
         ?      B       (?X)  :   0 :  0 ;
         0      X       (?X)  :   0 :  0 ;
    

        endtable

endprimitive


primitive CORE65GPSVT_U_FD_P_SN (Q, D, CP, SN);
        output Q;
        input  D, CP, SN;
        reg    Q;

        table
        //D     CP      SN      : Qn: Qn+1
         ?      ?       0       : ? : 1 ;
         *      ?       0       : ? : 1 ;
         ?      *       0       : ? : 1 ;
         1      (01)    1       : ? : 1 ;
         0      (01)    1       : ? : 0 ;
         *      B       1       : ? : - ;
         ?      (?0)    1       : ? : - ;
         0      (1X)    1       : 0 : 0 ;
         1      (1X)    1       : 1 : 1 ;
         ?       B     (?1)     : ? : - ;
         0      x      (?1)     : 0 : 0 ;
         0      (0X)    1       : 0 : 0 ;
         0      (X1)    1       : 0 : 0 ;
         1      (0X)    1       : 1 : 1 ;
         1      (X1)    1       : 1 : 1 ;
         1      (01)    X       : ? : 1 ;
         1      (0x)    X       : 1 : 1 ;
         ?      B       (?X)    : 1 : 1 ;
         1      X       (?X)    : 1 : 1 ;


        endtable

endprimitive


primitive CORE65GPSVT_U_LD_P (Q, D, G);
        output Q;
        input  D, G;
        reg    Q;
 
        table
        //D     G       : Qn : Qn+1
         0      1       :  ? :  0 ;
         1      1       :  ? :  1 ;
         X      1       :  ? :  X ;
         ?      0       :  ? :  - ;
         0      X       :  0 :  0 ;
         1      X       :  1 :  1 ;
 
 
        endtable
 
endprimitive


primitive CORE65GPSVT_U_LD_P_RN (Q, D, G, RN);
        output Q;
        input  D, G, RN;
        reg    Q;

        table
        //D     G       RN      : Qn : Qn+1
         ?      ?       0       :  ? :  0 ;
         0      1       1       :  ? :  0 ;
         1      1       1       :  ? :  1 ;
         X      1       1       :  ? :  X ;
         ?      0       1       :  ? :  - ;
         0      X       1       :  0 :  0 ;
         1      X       1       :  1 :  1 ;
         0      X       X       :  0 :  0 ;
         X      0       X       :  0 :  0 ;
         1      0       X       :  0 :  0 ;
         0      ?       X       :  0 :  0 ;
         0      1       X       :  ? :  0 ;


        endtable

endprimitive



primitive CORE65GPSVT_U_MUX2 (Z, A, B, S);

   output Z;
   input  A, B, S;

   table

      // A  B  S  :  Z

         0  ?  0  :  0  ;
         1  ?  0  :  1  ;

         ?  0  1  :  0  ;
         ?  1  1  :  1  ;

      // Cases reducing pessimism

         0  0  x  :  0  ;
         1  1  x  :  1  ;

   endtable


endprimitive
