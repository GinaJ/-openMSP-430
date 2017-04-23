
module omsp_sync_reset ( rst_s, clk, rst_a );
  input clk, rst_a;
  output rst_s;
  wire   data_sync_0_, n1;

  HS65_GS_DFPSQX9 data_sync_reg_0_ ( .D(1'b0), .CP(clk), .SN(n1), .Q(
        data_sync_0_) );
  HS65_GS_DFPSQX9 data_sync_reg_1_ ( .D(data_sync_0_), .CP(clk), .SN(n1), .Q(
        rst_s) );
  HS65_GS_IVX9 U3 ( .A(rst_a), .Z(n1) );
endmodule


module omsp_scan_mux_0 ( data_out, data_in_scan, data_in_func, scan_mode );
  input data_in_scan, data_in_func, scan_mode;
  output data_out;
  wire   n1;

  HS65_GS_AO22X9 U1 ( .A(scan_mode), .B(data_in_scan), .C(data_in_func), .D(n1), .Z(data_out) );
  HS65_GS_IVX9 U2 ( .A(scan_mode), .Z(n1) );
endmodule


module omsp_scan_mux_2 ( data_out, data_in_scan, data_in_func, scan_mode );
  input data_in_scan, data_in_func, scan_mode;
  output data_out;
  wire   n1;

  HS65_GS_AO22X9 U1 ( .A(scan_mode), .B(data_in_scan), .C(data_in_func), .D(n1), .Z(data_out) );
  HS65_GS_IVX9 U2 ( .A(scan_mode), .Z(n1) );
endmodule


module omsp_sync_cell ( data_out, clk, data_in, rst );
  input clk, data_in, rst;
  output data_out;
  wire   data_sync_0_, n1;

  HS65_GS_DFPRQX9 data_sync_reg_0_ ( .D(data_in), .CP(clk), .RN(n1), .Q(
        data_sync_0_) );
  HS65_GS_DFPRQX4 data_sync_reg_1_ ( .D(data_sync_0_), .CP(clk), .RN(n1), .Q(
        data_out) );
  HS65_GS_IVX9 U3 ( .A(rst), .Z(n1) );
endmodule


module omsp_scan_mux_1 ( data_out, data_in_scan, data_in_func, scan_mode );
  input data_in_scan, data_in_func, scan_mode;
  output data_out;
  wire   n1;

  HS65_GS_AO22X9 U1 ( .A(scan_mode), .B(data_in_scan), .C(data_in_func), .D(n1), .Z(data_out) );
  HS65_GS_IVX9 U2 ( .A(scan_mode), .Z(n1) );
endmodule


module omsp_clock_module ( aclk, aclk_en, cpu_en_s, cpu_mclk, dma_mclk, 
        dbg_clk, dbg_en_s, dbg_rst, dco_enable, dco_wkup, lfxt_enable, 
        lfxt_wkup, per_dout, por, puc_pnd_set, puc_rst, smclk, smclk_en, 
        cpu_en, cpuoff, dbg_cpu_reset, dbg_en, dco_clk, lfxt_clk, 
        mclk_dma_enable, mclk_dma_wkup, mclk_enable, mclk_wkup, oscoff, 
        per_addr, per_din, per_en, per_we, reset_n, scan_enable, scan_mode, 
        scg0, scg1, wdt_reset );
  output [15:0] per_dout;
  input [13:0] per_addr;
  input [15:0] per_din;
  input [1:0] per_we;
  input cpu_en, cpuoff, dbg_cpu_reset, dbg_en, dco_clk, lfxt_clk,
         mclk_dma_enable, mclk_dma_wkup, mclk_enable, mclk_wkup, oscoff,
         per_en, reset_n, scan_enable, scan_mode, scg0, scg1, wdt_reset;
  output aclk, aclk_en, cpu_en_s, cpu_mclk, dma_mclk, dbg_clk, dbg_en_s,
         dbg_rst, dco_enable, dco_wkup, lfxt_enable, lfxt_wkup, por,
         puc_pnd_set, puc_rst, smclk, smclk_en;
  wire   por_noscan, puc_a, puc_noscan_n, puc_a_scan, n2, n3, n54;

  omsp_sync_reset sync_reset_por ( .rst_s(por_noscan), .clk(dco_clk), .rst_a(
        n3) );
  omsp_scan_mux_0 scan_mux_por ( .data_out(por), .data_in_scan(n3), 
        .data_in_func(por_noscan), .scan_mode(scan_mode) );
  omsp_scan_mux_2 scan_mux_puc_rst_a ( .data_out(puc_a_scan), .data_in_scan(n3), .data_in_func(puc_a), .scan_mode(scan_mode) );
  omsp_sync_cell sync_cell_puc ( .data_out(puc_noscan_n), .clk(dco_clk), 
        .data_in(n2), .rst(puc_a_scan) );
  omsp_scan_mux_1 scan_mux_puc_rst ( .data_out(puc_rst), .data_in_scan(n3), 
        .data_in_func(puc_pnd_set), .scan_mode(scan_mode) );
  HS65_GS_IVX9 U19 ( .A(1'b0), .Z(smclk_en) );
  HS65_GS_IVX9 U21 ( .A(1'b1), .Z(per_dout[0]) );
  HS65_GS_IVX9 U23 ( .A(1'b1), .Z(per_dout[1]) );
  HS65_GS_IVX9 U25 ( .A(1'b1), .Z(per_dout[2]) );
  HS65_GS_IVX9 U27 ( .A(1'b1), .Z(per_dout[3]) );
  HS65_GS_IVX9 U29 ( .A(1'b1), .Z(per_dout[4]) );
  HS65_GS_IVX9 U31 ( .A(1'b1), .Z(per_dout[5]) );
  HS65_GS_IVX9 U33 ( .A(1'b1), .Z(per_dout[6]) );
  HS65_GS_IVX9 U35 ( .A(1'b1), .Z(per_dout[7]) );
  HS65_GS_IVX9 U37 ( .A(1'b1), .Z(per_dout[8]) );
  HS65_GS_IVX9 U39 ( .A(1'b1), .Z(per_dout[9]) );
  HS65_GS_IVX9 U41 ( .A(1'b1), .Z(per_dout[10]) );
  HS65_GS_IVX9 U43 ( .A(1'b1), .Z(per_dout[11]) );
  HS65_GS_IVX9 U45 ( .A(1'b1), .Z(per_dout[12]) );
  HS65_GS_IVX9 U47 ( .A(1'b1), .Z(per_dout[13]) );
  HS65_GS_IVX9 U49 ( .A(1'b1), .Z(per_dout[14]) );
  HS65_GS_IVX9 U51 ( .A(1'b1), .Z(per_dout[15]) );
  HS65_GS_IVX9 U53 ( .A(1'b1), .Z(lfxt_wkup) );
  HS65_GS_IVX9 U55 ( .A(1'b0), .Z(lfxt_enable) );
  HS65_GS_IVX9 U57 ( .A(1'b0), .Z(dco_wkup) );
  HS65_GS_IVX9 U59 ( .A(1'b0), .Z(dco_enable) );
  HS65_GS_IVX9 U61 ( .A(1'b0), .Z(dbg_rst) );
  HS65_GS_IVX9 U63 ( .A(1'b1), .Z(dbg_en_s) );
  HS65_GS_IVX9 U65 ( .A(1'b1), .Z(dbg_clk) );
  HS65_GS_IVX9 U67 ( .A(1'b0), .Z(aclk_en) );
  HS65_GS_IVX9 U69 ( .A(puc_noscan_n), .Z(puc_pnd_set) );
  HS65_GS_IVX9 U70 ( .A(dco_clk), .Z(n54) );
  HS65_GS_IVX9 U71 ( .A(n54), .Z(smclk) );
  HS65_GS_IVX9 U72 ( .A(n54), .Z(aclk) );
  HS65_GS_IVX27 U73 ( .A(n54), .Z(cpu_mclk) );
  HS65_GS_IVX9 U74 ( .A(n54), .Z(dma_mclk) );
  HS65_GS_IVX9 U75 ( .A(reset_n), .Z(n3) );
  HS65_GS_BFX9 U76 ( .A(cpu_en), .Z(cpu_en_s) );
  HS65_GS_OR2X9 U77 ( .A(por), .B(wdt_reset), .Z(puc_a) );
  HS65_GS_IVX9 U78 ( .A(dbg_cpu_reset), .Z(n2) );
endmodule


module omsp_frontend ( OBSERVE_nmi_pnd, OBSERVE_i_state, OBSERVE_e_state, 
        OBSERVE_decode, OBSERVE_ir, OBSERVE_irq_detect, OBSERVE_irq_num, 
        OBSERVE_pc, cpu_halt_st, decode_noirq, e_state, exec_done, inst_ad, 
        inst_as, inst_alu, inst_bw, inst_dest, inst_dext, inst_irq_rst, 
        inst_jmp, inst_mov, inst_sext, inst_so, inst_src, inst_type, irq_acc, 
        mab, mb_en, mclk_dma_enable, mclk_dma_wkup, mclk_enable, mclk_wkup, 
        nmi_acc, pc, pc_nxt, cpu_en_s, cpu_halt_cmd, cpuoff, dbg_reg_sel, 
        dma_en, dma_wkup, fe_pmem_wait, gie, irq, mclk, mdb_in, nmi_pnd, 
        nmi_wkup, pc_sw, pc_sw_wr, puc_rst, scan_enable, wdt_irq, wdt_wkup, 
        wkup );
  output [2:0] OBSERVE_i_state;
  output [3:0] OBSERVE_e_state;
  output [15:0] OBSERVE_ir;
  output [3:0] OBSERVE_irq_num;
  output [15:0] OBSERVE_pc;
  output [3:0] e_state;
  output [7:0] inst_ad;
  output [7:0] inst_as;
  output [11:0] inst_alu;
  output [15:0] inst_dest;
  output [15:0] inst_dext;
  output [7:0] inst_jmp;
  output [15:0] inst_sext;
  output [7:0] inst_so;
  output [15:0] inst_src;
  output [2:0] inst_type;
  output [13:0] irq_acc;
  output [15:0] mab;
  output [15:0] pc;
  output [15:0] pc_nxt;
  input [3:0] dbg_reg_sel;
  input [13:0] irq;
  input [15:0] mdb_in;
  input [15:0] pc_sw;
  input cpu_en_s, cpu_halt_cmd, cpuoff, dma_en, dma_wkup, fe_pmem_wait, gie,
         mclk, nmi_pnd, nmi_wkup, pc_sw_wr, puc_rst, scan_enable, wdt_irq,
         wdt_wkup, wkup;
  output OBSERVE_nmi_pnd, OBSERVE_decode, OBSERVE_irq_detect, cpu_halt_st,
         decode_noirq, exec_done, inst_bw, inst_irq_rst, inst_mov, mb_en,
         mclk_dma_enable, mclk_dma_wkup, mclk_enable, mclk_wkup, nmi_acc;
  wire   fetch, N249, pc_incr_15_, pc_incr_14_, pc_incr_13_, pc_incr_12_,
         pc_incr_11_, pc_incr_10_, pc_incr_9_, pc_incr_8_, pc_incr_7_,
         pc_incr_6_, pc_incr_5_, pc_incr_4_, pc_incr_3_, pc_incr_2_,
         pc_incr_1_, pmem_busy, N702, ext_nxt_15_, ext_nxt_14_, ext_nxt_13_,
         ext_nxt_12_, ext_nxt_11_, ext_nxt_10_, ext_nxt_9_, ext_nxt_8_,
         ext_nxt_7_, ext_nxt_6_, ext_nxt_5_, ext_nxt_4_, ext_nxt_3_,
         ext_nxt_2_, ext_nxt_1_, exec_jmp, exec_dst_wr, exec_src_wr,
         exec_dext_rdy, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19,
         n20, n21, n22, n24, n25, n28, n29, n31, n32, n33, n36, n38, n39, n40,
         n41, n43, n44, n45, n48, n49, n50, n52, n53, n54, n57, n58, n59, n60,
         n61, n62, n65, n66, n67, n68, n69, n70, n71, n73, n77, n80, n81, n82,
         n85, n88, n91, n93, n95, n96, n98, n99, n100, n101, n102, n103, n104,
         n106, n108, n110, n111, n114, n115, n117, n118, n119, n121, n122,
         n123, n124, n125, n126, n127, n130, n132, n133, n140, n142, n143,
         n145, n148, n149, n151, n152, n153, n154, n156, n157, n159, n160,
         n161, n163, n164, n166, n167, n168, n169, n170, n171, n173, n174,
         n176, n177, n178, n179, n182, n183, n197, n199, n200, n201, n202,
         n203, n204, n205, n206, n207, n208, n209, n210, n211, n212, n213,
         n214, n215, n216, n217, n218, n219, n220, n221, n222, n223, n224,
         n225, n226, n227, n228, n229, n230, n231, n232, n233, n234, n235,
         n236, n237, n238, n239, n240, n242, n246, n247, n249, n250, n251,
         n252, n253, n254, n255, n256, n257, n259, n260, n262, n263, n264,
         n265, n266, n268, n269, n270, n271, n272, n273, n274, n275, n276,
         n277, n280, n281, n282, n283, n284, n285, n287, n290, n291, n292,
         n293, n295, n297, n299, n300, n301, n302, n303, n306, n307, n308,
         n310, n312, n313, n315, n316, n317, n319, n320, n321, n322, n323,
         n324, n325, n326, n327, n328, n329, n330, n332, n333, n334, n335,
         n336, n337, n338, n340, n341, n342, n343, n344, n345, n346, n347,
         n348, n349, n350, n351, n352, n353, n354, n355, n356, n357, n358,
         n359, n360, n361, n362, n363, n364, n365, n366, n367, n368, n369,
         n370, n371, n372, n373, n374, n375, n376, n377, n378, n379, n380,
         n381, n382, n383, n384, n385, n386, n387, n388, n389, n390, n391,
         n392, n393, n394, n395, n396, n397, n398, n399, n400, n401, n402,
         n403, n404, n405, n406, n407, n408, n409, n410, n411, n412, n413,
         n414, n415, n416, n417, n418, n419, n420, n421, n422, n423, n424,
         n425, n426, n427, n428, n429, n46, n187, n188, n189, n190, n191, n192,
         n193, n194, n195, n198, n241, n243, n244, n245, n248, n258, n261,
         n267, n278, n279, n286, n288, n289, n294, n296, n298, n304, n305,
         n309, n311, n314, n318, n331, n339, n430, n431, n432, n433, n434,
         n435, n436, n437, n438, n439, n440, n441, n442, n443, n444, n445,
         n446, n447, n448, n449, n450, n451, n452, n453, n454, n455, n456,
         n457, n458, n459, n460, n461, n462, n463, n464, n465, n466, n467,
         n468, n469, n470, n471, n472, n473, n474, n475, n476, n477, n478,
         n479, n480, n481, n482, n483, n484, n485, n486, n487, n488, n489,
         n490, n491, n492, n493, n494, n495, n496, n497, n498, n499;
  wire   [2:0] i_state_nxt;
  wire   [1:0] inst_sz;
  wire   [3:0] e_state_nxt;
  wire   [2:0] inst_jmp_bin;
  wire   [3:0] inst_dest_bin;
  wire   [3:0] inst_src_bin;
  wire   [15:2] add_477_carry;
  wire   [15:2] add_428_carry;

  HS65_GS_DFPSQX9 inst_irq_rst_reg ( .D(n338), .CP(mclk), .SN(n192), .Q(
        inst_irq_rst) );
  HS65_GS_DFPSQX9 irq_num_reg_3_ ( .D(n413), .CP(mclk), .SN(n191), .Q(
        OBSERVE_irq_num[3]) );
  HS65_GS_DFPSQX9 irq_num_reg_2_ ( .D(n414), .CP(mclk), .SN(n191), .Q(
        OBSERVE_irq_num[2]) );
  HS65_GS_DFPSQX9 irq_num_reg_1_ ( .D(n415), .CP(mclk), .SN(n191), .Q(
        OBSERVE_irq_num[1]) );
  HS65_GS_DFPSQX9 irq_num_reg_0_ ( .D(n416), .CP(mclk), .SN(n191), .Q(
        OBSERVE_irq_num[0]) );
  HS65_GS_DFPRQNX9 inst_so_reg_2_ ( .D(n408), .CP(mclk), .RN(n192), .QN(n332)
         );
  HS65_GS_DFPRQNX9 inst_so_reg_3_ ( .D(n407), .CP(mclk), .RN(n192), .QN(n333)
         );
  HS65_GS_DFPRQNX9 inst_ad_reg_1_ ( .D(n388), .CP(mclk), .RN(n192), .QN(n337)
         );
  HS65_GS_DFPRQNX9 inst_as_reg_2_ ( .D(n399), .CP(mclk), .RN(n192), .QN(n334)
         );
  HS65_GS_DFPRQNX9 inst_as_reg_1_ ( .D(n400), .CP(mclk), .RN(n192), .QN(n335)
         );
  HS65_GS_DFPRQNX9 inst_as_reg_4_ ( .D(n397), .CP(mclk), .RN(n192), .QN(n336)
         );
  HS65_GS_DFPSQX9 e_state_reg_0_ ( .D(e_state_nxt[0]), .CP(mclk), .SN(n191), 
        .Q(e_state[0]) );
  HS65_GS_DFPRQX4 inst_dext_reg_0_ ( .D(n386), .CP(mclk), .RN(n431), .Q(
        inst_dext[0]) );
  HS65_GS_DFPRQX4 pc_reg_0_ ( .D(pc_nxt[0]), .CP(mclk), .RN(n431), .Q(pc[0])
         );
  HS65_GS_DFPRQX4 i_state_reg_1_ ( .D(i_state_nxt[1]), .CP(mclk), .RN(n187), 
        .Q(OBSERVE_i_state[1]) );
  HS65_GS_DFPRQX4 i_state_reg_2_ ( .D(i_state_nxt[2]), .CP(mclk), .RN(n187), 
        .Q(OBSERVE_i_state[2]) );
  HS65_GS_DFPRQX4 inst_src_bin_reg_3_ ( .D(n417), .CP(mclk), .RN(n431), .Q(
        inst_src_bin[3]) );
  HS65_GS_DFPRQX4 inst_src_bin_reg_2_ ( .D(n418), .CP(mclk), .RN(n431), .Q(
        inst_src_bin[2]) );
  HS65_GS_DFPRQX4 inst_src_bin_reg_1_ ( .D(n419), .CP(mclk), .RN(n191), .Q(
        inst_src_bin[1]) );
  HS65_GS_DFPRQX4 inst_so_reg_7_ ( .D(n403), .CP(mclk), .RN(n431), .Q(
        inst_so[7]) );
  HS65_GS_DFPRQX4 inst_dest_bin_reg_3_ ( .D(n421), .CP(mclk), .RN(n190), .Q(
        inst_dest_bin[3]) );
  HS65_GS_DFPRQX4 inst_dest_bin_reg_2_ ( .D(n422), .CP(mclk), .RN(n191), .Q(
        inst_dest_bin[2]) );
  HS65_GS_DFPRQX4 inst_dest_bin_reg_0_ ( .D(n424), .CP(mclk), .RN(n189), .Q(
        inst_dest_bin[0]) );
  HS65_GS_DFPRQX4 inst_so_reg_6_ ( .D(n404), .CP(mclk), .RN(n431), .Q(
        inst_so[6]) );
  HS65_GS_DFPRQX4 inst_alu_reg_10_ ( .D(n350), .CP(mclk), .RN(n188), .Q(
        inst_alu[10]) );
  HS65_GS_DFPRQX4 inst_type_reg_0_ ( .D(n412), .CP(mclk), .RN(n431), .Q(
        inst_type[0]) );
  HS65_GS_DFPRQX4 inst_type_reg_2_ ( .D(n391), .CP(mclk), .RN(n188), .Q(
        inst_type[2]) );
  HS65_GS_DFPRQX4 inst_src_bin_reg_0_ ( .D(n420), .CP(mclk), .RN(n188), .Q(
        inst_src_bin[0]) );
  HS65_GS_DFPRQX4 inst_jmp_bin_reg_1_ ( .D(n426), .CP(mclk), .RN(n431), .Q(
        inst_jmp_bin[1]) );
  HS65_GS_DFPRQX4 inst_jmp_bin_reg_0_ ( .D(n427), .CP(mclk), .RN(n192), .Q(
        inst_jmp_bin[0]) );
  HS65_GS_DFPRQX4 inst_dest_bin_reg_1_ ( .D(n423), .CP(mclk), .RN(n191), .Q(
        inst_dest_bin[1]) );
  HS65_GS_DFPRQX4 inst_alu_reg_1_ ( .D(n341), .CP(mclk), .RN(n187), .Q(
        inst_alu[1]) );
  HS65_GS_DFPRQX4 inst_jmp_bin_reg_2_ ( .D(n425), .CP(mclk), .RN(n431), .Q(
        inst_jmp_bin[2]) );
  HS65_GS_DFPRQX4 inst_alu_reg_11_ ( .D(n351), .CP(mclk), .RN(n188), .Q(
        inst_alu[11]) );
  HS65_GS_DFPRQX4 inst_ad_reg_0_ ( .D(n389), .CP(mclk), .RN(n188), .Q(
        inst_ad[0]) );
  HS65_GS_DFPRQX4 inst_ad_reg_6_ ( .D(n354), .CP(mclk), .RN(n188), .Q(
        inst_ad[6]) );
  HS65_GS_DFPRQX4 inst_ad_reg_4_ ( .D(n387), .CP(mclk), .RN(n187), .Q(
        inst_ad[4]) );
  HS65_GS_DFPRQX4 inst_bw_reg ( .D(n392), .CP(mclk), .RN(n188), .Q(inst_bw) );
  HS65_GS_DFPRQX4 inst_mov_reg ( .D(n390), .CP(mclk), .RN(n189), .Q(inst_mov)
         );
  HS65_GS_DFPRQX4 inst_alu_reg_5_ ( .D(n345), .CP(mclk), .RN(n191), .Q(
        inst_alu[5]) );
  HS65_GS_DFPRQX4 inst_alu_reg_6_ ( .D(n346), .CP(mclk), .RN(n190), .Q(
        inst_alu[6]) );
  HS65_GS_DFPRQX4 inst_so_reg_5_ ( .D(n405), .CP(mclk), .RN(n191), .Q(
        inst_so[5]) );
  HS65_GS_DFPRQX4 inst_so_reg_4_ ( .D(n406), .CP(mclk), .RN(n191), .Q(
        inst_so[4]) );
  HS65_GS_DFPRQX4 inst_so_reg_1_ ( .D(n409), .CP(mclk), .RN(n431), .Q(
        inst_so[1]) );
  HS65_GS_DFPRQX4 inst_so_reg_0_ ( .D(n410), .CP(mclk), .RN(n431), .Q(
        inst_so[0]) );
  HS65_GS_DFPRQX4 inst_type_reg_1_ ( .D(n402), .CP(mclk), .RN(n190), .Q(
        inst_type[1]) );
  HS65_GS_DFPRQX4 inst_alu_reg_8_ ( .D(n348), .CP(mclk), .RN(n187), .Q(
        inst_alu[8]) );
  HS65_GS_DFPRQX4 inst_alu_reg_7_ ( .D(n347), .CP(mclk), .RN(n187), .Q(
        inst_alu[7]) );
  HS65_GS_DFPRQX4 inst_as_reg_6_ ( .D(n395), .CP(mclk), .RN(n191), .Q(
        inst_as[6]) );
  HS65_GS_DFPRQX4 inst_as_reg_5_ ( .D(n396), .CP(mclk), .RN(n190), .Q(
        inst_as[5]) );
  HS65_GS_DFPRQX4 inst_as_reg_3_ ( .D(n398), .CP(mclk), .RN(n192), .Q(
        inst_as[3]) );
  HS65_GS_DFPRQX4 inst_dext_reg_1_ ( .D(n385), .CP(mclk), .RN(n431), .Q(
        inst_dext[1]) );
  HS65_GS_DFPRQX4 inst_alu_reg_9_ ( .D(n349), .CP(mclk), .RN(n188), .Q(
        inst_alu[9]) );
  HS65_GS_DFPRQX4 inst_alu_reg_2_ ( .D(n342), .CP(mclk), .RN(n187), .Q(
        inst_alu[2]) );
  HS65_GS_DFPRQX4 inst_alu_reg_4_ ( .D(n344), .CP(mclk), .RN(n189), .Q(
        inst_alu[4]) );
  HS65_GS_DFPRQX4 inst_alu_reg_0_ ( .D(n340), .CP(mclk), .RN(n189), .Q(
        inst_alu[0]) );
  HS65_GS_DFPRQX4 inst_alu_reg_3_ ( .D(n343), .CP(mclk), .RN(n188), .Q(
        inst_alu[3]) );
  HS65_GS_DFPRQX4 exec_jmp_reg ( .D(n428), .CP(mclk), .RN(n187), .Q(exec_jmp)
         );
  HS65_GS_DFPRQX4 inst_dext_reg_2_ ( .D(n384), .CP(mclk), .RN(n188), .Q(
        inst_dext[2]) );
  HS65_GS_DFPRQX4 inst_sz_reg_1_ ( .D(n353), .CP(mclk), .RN(n187), .Q(
        inst_sz[1]) );
  HS65_GS_DFPRQX4 inst_dext_reg_3_ ( .D(n383), .CP(mclk), .RN(n190), .Q(
        inst_dext[3]) );
  HS65_GS_DFPRQX4 inst_as_reg_0_ ( .D(n401), .CP(mclk), .RN(n189), .Q(
        inst_as[0]) );
  HS65_GS_DFPRQX4 inst_sz_reg_0_ ( .D(n352), .CP(mclk), .RN(n191), .Q(
        inst_sz[0]) );
  HS65_GS_DFPRQX4 i_state_reg_0_ ( .D(i_state_nxt[0]), .CP(mclk), .RN(n191), 
        .Q(OBSERVE_i_state[0]) );
  HS65_GS_DFPRQX4 e_state_reg_2_ ( .D(e_state_nxt[2]), .CP(mclk), .RN(n431), 
        .Q(e_state[2]) );
  HS65_GS_DFPRQX4 inst_as_reg_7_ ( .D(n393), .CP(mclk), .RN(n187), .Q(
        inst_as[7]) );
  HS65_GS_DFPRQX4 inst_dext_reg_4_ ( .D(n382), .CP(mclk), .RN(n192), .Q(
        inst_dext[4]) );
  HS65_GS_DFPRQX4 e_state_reg_1_ ( .D(e_state_nxt[1]), .CP(mclk), .RN(n191), 
        .Q(e_state[1]) );
  HS65_GS_DFPRQX4 cpu_halt_st_reg ( .D(N249), .CP(mclk), .RN(n191), .Q(
        cpu_halt_st) );
  HS65_GS_DFPRQX4 e_state_reg_3_ ( .D(e_state_nxt[3]), .CP(mclk), .RN(n431), 
        .Q(e_state[3]) );
  HS65_GS_DFPRQX4 inst_dext_reg_5_ ( .D(n381), .CP(mclk), .RN(n190), .Q(
        inst_dext[5]) );
  HS65_GS_DFPRQX4 inst_dext_reg_6_ ( .D(n380), .CP(mclk), .RN(n190), .Q(
        inst_dext[6]) );
  HS65_GS_DFPRQX4 pc_reg_1_ ( .D(mab[1]), .CP(mclk), .RN(n187), .Q(pc[1]) );
  HS65_GS_DFPRQX4 inst_dext_reg_7_ ( .D(n379), .CP(mclk), .RN(n190), .Q(
        inst_dext[7]) );
  HS65_GS_DFPRQX4 inst_sext_reg_1_ ( .D(n369), .CP(mclk), .RN(n189), .Q(
        inst_sext[1]) );
  HS65_GS_DFPRQX4 inst_sext_reg_3_ ( .D(n367), .CP(mclk), .RN(n189), .Q(
        inst_sext[3]) );
  HS65_GS_DFPRQX4 inst_sext_reg_2_ ( .D(n368), .CP(mclk), .RN(n189), .Q(
        inst_sext[2]) );
  HS65_GS_DFPRQX4 pc_reg_2_ ( .D(mab[2]), .CP(mclk), .RN(n187), .Q(pc[2]) );
  HS65_GS_DFPRQX4 inst_sext_reg_6_ ( .D(n364), .CP(mclk), .RN(n189), .Q(
        inst_sext[6]) );
  HS65_GS_DFPRQX4 inst_sext_reg_5_ ( .D(n365), .CP(mclk), .RN(n189), .Q(
        inst_sext[5]) );
  HS65_GS_DFPRQX4 inst_sext_reg_4_ ( .D(n366), .CP(mclk), .RN(n189), .Q(
        inst_sext[4]) );
  HS65_GS_DFPRQX4 inst_sext_reg_0_ ( .D(n370), .CP(mclk), .RN(n189), .Q(
        inst_sext[0]) );
  HS65_GS_DFPRQX4 inst_sext_reg_7_ ( .D(n363), .CP(mclk), .RN(n189), .Q(
        inst_sext[7]) );
  HS65_GS_DFPRQX4 pc_reg_3_ ( .D(mab[3]), .CP(mclk), .RN(n431), .Q(pc[3]) );
  HS65_GS_DFPRQX4 inst_dext_reg_8_ ( .D(n378), .CP(mclk), .RN(n190), .Q(
        inst_dext[8]) );
  HS65_GS_DFPRQX4 inst_sext_reg_8_ ( .D(n362), .CP(mclk), .RN(n189), .Q(
        inst_sext[8]) );
  HS65_GS_DFPRQX4 pc_reg_4_ ( .D(mab[4]), .CP(mclk), .RN(n188), .Q(pc[4]) );
  HS65_GS_DFPRQX4 inst_dext_reg_9_ ( .D(n377), .CP(mclk), .RN(n190), .Q(
        inst_dext[9]) );
  HS65_GS_DFPRQX4 pc_reg_5_ ( .D(mab[5]), .CP(mclk), .RN(n187), .Q(pc[5]) );
  HS65_GS_DFPRQX4 inst_sext_reg_9_ ( .D(n361), .CP(mclk), .RN(n189), .Q(
        inst_sext[9]) );
  HS65_GS_DFPRQX4 pc_reg_6_ ( .D(mab[6]), .CP(mclk), .RN(n188), .Q(pc[6]) );
  HS65_GS_DFPRQX4 inst_dext_reg_10_ ( .D(n376), .CP(mclk), .RN(n190), .Q(
        inst_dext[10]) );
  HS65_GS_DFPRQX4 inst_sext_reg_10_ ( .D(n360), .CP(mclk), .RN(n189), .Q(
        inst_sext[10]) );
  HS65_GS_DFPRQX4 pc_reg_7_ ( .D(mab[7]), .CP(mclk), .RN(n187), .Q(pc[7]) );
  HS65_GS_DFPRQX4 pc_reg_8_ ( .D(mab[8]), .CP(mclk), .RN(n188), .Q(pc[8]) );
  HS65_GS_DFPRQX4 inst_dext_reg_11_ ( .D(n375), .CP(mclk), .RN(n190), .Q(
        inst_dext[11]) );
  HS65_GS_DFPRQX4 inst_sext_reg_11_ ( .D(n359), .CP(mclk), .RN(n192), .Q(
        inst_sext[11]) );
  HS65_GS_DFPRQX4 pc_reg_9_ ( .D(mab[9]), .CP(mclk), .RN(n187), .Q(pc[9]) );
  HS65_GS_DFPRQX4 inst_dext_reg_12_ ( .D(n374), .CP(mclk), .RN(n190), .Q(
        inst_dext[12]) );
  HS65_GS_DFPRQX4 inst_sext_reg_12_ ( .D(n358), .CP(mclk), .RN(n192), .Q(
        inst_sext[12]) );
  HS65_GS_DFPRQX4 pc_reg_10_ ( .D(mab[10]), .CP(mclk), .RN(n188), .Q(pc[10])
         );
  HS65_GS_DFPRQX4 pc_reg_11_ ( .D(mab[11]), .CP(mclk), .RN(n187), .Q(pc[11])
         );
  HS65_GS_DFPRQX4 inst_dext_reg_13_ ( .D(n373), .CP(mclk), .RN(n190), .Q(
        inst_dext[13]) );
  HS65_GS_DFPRQX4 inst_sext_reg_13_ ( .D(n357), .CP(mclk), .RN(n192), .Q(
        inst_sext[13]) );
  HS65_GS_DFPRQX4 inst_dext_reg_14_ ( .D(n372), .CP(mclk), .RN(n190), .Q(
        inst_dext[14]) );
  HS65_GS_DFPRQX4 inst_sext_reg_14_ ( .D(n356), .CP(mclk), .RN(n192), .Q(
        inst_sext[14]) );
  HS65_GS_DFPRQX4 pc_reg_12_ ( .D(mab[12]), .CP(mclk), .RN(n188), .Q(pc[12])
         );
  HS65_GS_DFPRQX4 inst_dext_reg_15_ ( .D(n371), .CP(mclk), .RN(n188), .Q(
        inst_dext[15]) );
  HS65_GS_DFPRQX4 inst_sext_reg_15_ ( .D(n355), .CP(mclk), .RN(n192), .Q(
        inst_sext[15]) );
  HS65_GS_DFPRQX4 pc_reg_13_ ( .D(mab[13]), .CP(mclk), .RN(n187), .Q(pc[13])
         );
  HS65_GS_DFPRQX4 pc_reg_14_ ( .D(mab[14]), .CP(mclk), .RN(n188), .Q(pc[14])
         );
  HS65_GS_DFPRQX4 pc_reg_15_ ( .D(mab[15]), .CP(mclk), .RN(n187), .Q(pc[15])
         );
  HS65_GS_DFPRQX4 pmem_busy_reg ( .D(fe_pmem_wait), .CP(mclk), .RN(n192), .Q(
        pmem_busy) );
  HS65_GS_DFPRQX4 exec_dst_wr_reg ( .D(n429), .CP(mclk), .RN(n431), .Q(
        exec_dst_wr) );
  HS65_GS_DFPRQX4 exec_src_wr_reg ( .D(n411), .CP(mclk), .RN(n431), .Q(
        exec_src_wr) );
  HS65_GS_DFPRQX4 exec_dext_rdy_reg ( .D(n394), .CP(mclk), .RN(n431), .Q(
        exec_dext_rdy) );
  HS65_GS_IVX9 U3 ( .A(1'b0), .Z(mclk_wkup) );
  HS65_GS_IVX9 U5 ( .A(1'b0), .Z(mclk_enable) );
  HS65_GS_IVX9 U7 ( .A(1'b0), .Z(mclk_dma_wkup) );
  HS65_GS_IVX9 U9 ( .A(1'b0), .Z(mclk_dma_enable) );
  HS65_GS_IVX9 U11 ( .A(1'b1), .Z(inst_ad[2]) );
  HS65_GS_IVX9 U13 ( .A(1'b1), .Z(inst_ad[3]) );
  HS65_GS_IVX9 U15 ( .A(1'b1), .Z(inst_ad[5]) );
  HS65_GS_IVX9 U17 ( .A(1'b1), .Z(inst_ad[7]) );
  HS65_GS_MUX21I1X6 U19 ( .D0(n257), .D1(n145), .S0(exec_src_wr), .Z(n329) );
  HS65_GS_MUX21X4 U20 ( .D0(n142), .D1(exec_src_wr), .S0(n140), .Z(n411) );
  HS65_GS_IVX9 U21 ( .A(n451), .Z(n46) );
  HS65_GS_BFX9 U22 ( .A(pc[1]), .Z(OBSERVE_pc[1]) );
  HS65_GS_BFX9 U23 ( .A(pc[2]), .Z(OBSERVE_pc[2]) );
  HS65_GS_BFX9 U24 ( .A(pc[3]), .Z(OBSERVE_pc[3]) );
  HS65_GS_BFX9 U25 ( .A(pc[4]), .Z(OBSERVE_pc[4]) );
  HS65_GS_BFX9 U26 ( .A(pc[5]), .Z(OBSERVE_pc[5]) );
  HS65_GS_BFX9 U27 ( .A(pc[6]), .Z(OBSERVE_pc[6]) );
  HS65_GS_BFX9 U28 ( .A(pc[7]), .Z(OBSERVE_pc[7]) );
  HS65_GS_BFX9 U29 ( .A(pc[8]), .Z(OBSERVE_pc[8]) );
  HS65_GS_BFX9 U30 ( .A(pc[9]), .Z(OBSERVE_pc[9]) );
  HS65_GS_BFX9 U31 ( .A(pc[10]), .Z(OBSERVE_pc[10]) );
  HS65_GS_BFX9 U32 ( .A(pc[11]), .Z(OBSERVE_pc[11]) );
  HS65_GS_BFX9 U33 ( .A(pc[12]), .Z(OBSERVE_pc[12]) );
  HS65_GS_BFX9 U34 ( .A(pc[13]), .Z(OBSERVE_pc[13]) );
  HS65_GS_BFX9 U35 ( .A(pc[14]), .Z(OBSERVE_pc[14]) );
  HS65_GS_BFX9 U36 ( .A(pc[15]), .Z(OBSERVE_pc[15]) );
  HS65_GS_BFX9 U37 ( .A(pc[0]), .Z(OBSERVE_pc[0]) );
  HS65_GS_BFX9 U38 ( .A(e_state[0]), .Z(OBSERVE_e_state[0]) );
  HS65_GS_BFX9 U39 ( .A(e_state[2]), .Z(OBSERVE_e_state[2]) );
  HS65_GS_BFX9 U40 ( .A(e_state[1]), .Z(OBSERVE_e_state[1]) );
  HS65_GS_BFX9 U41 ( .A(e_state[3]), .Z(OBSERVE_e_state[3]) );
  HS65_GS_IVX9 U42 ( .A(n337), .Z(inst_ad[1]) );
  HS65_GS_IVX9 U43 ( .A(n332), .Z(inst_so[2]) );
  HS65_GS_BFX9 U44 ( .A(nmi_pnd), .Z(OBSERVE_nmi_pnd) );
  HS65_GS_BFX9 U45 ( .A(n198), .Z(n195) );
  HS65_GS_OAI21X6 U46 ( .A(n336), .B(n456), .C(n301), .Z(N702) );
  HS65_GS_NOR4ABX2 U47 ( .A(n335), .B(n336), .C(inst_as[6]), .D(inst_as[5]), 
        .Z(n104) );
  HS65_GS_NAND2X7 U48 ( .A(n295), .B(n309), .Z(n127) );
  HS65_GS_NOR2X6 U49 ( .A(n124), .B(n118), .Z(n295) );
  HS65_GS_IVX9 U50 ( .A(n123), .Z(n309) );
  HS65_GS_IVX9 U51 ( .A(n198), .Z(n193) );
  HS65_GS_AOI112X4 U52 ( .A(n314), .B(n315), .C(n319), .D(n317), .Z(n124) );
  HS65_GS_NOR3AX2 U53 ( .A(n315), .B(n316), .C(n317), .Z(n118) );
  HS65_GS_NOR2AX3 U54 ( .A(n319), .B(n317), .Z(n123) );
  HS65_GS_NOR3X4 U55 ( .A(n315), .B(n316), .C(n321), .Z(n319) );
  HS65_GS_NAND3X5 U56 ( .A(n321), .B(n314), .C(n315), .Z(n117) );
  HS65_GS_IVX9 U57 ( .A(n281), .Z(n318) );
  HS65_GS_IVX9 U58 ( .A(n316), .Z(n314) );
  HS65_GS_IVX9 U59 ( .A(n260), .Z(n311) );
  HS65_GS_IVX9 U60 ( .A(n73), .Z(n286) );
  HS65_GS_IVX9 U61 ( .A(n81), .Z(n261) );
  HS65_GS_IVX9 U62 ( .A(n99), .Z(n278) );
  HS65_GS_IVX9 U63 ( .A(n195), .Z(n194) );
  HS65_GS_NAND2X7 U64 ( .A(n103), .B(n118), .Z(n93) );
  HS65_GS_IVX9 U65 ( .A(n38), .Z(n296) );
  HS65_GS_IVX9 U66 ( .A(n49), .Z(n298) );
  HS65_GS_BFX9 U67 ( .A(n195), .Z(OBSERVE_decode) );
  HS65_GS_BFX9 U68 ( .A(n187), .Z(n191) );
  HS65_GS_BFX9 U69 ( .A(n188), .Z(n189) );
  HS65_GS_BFX9 U70 ( .A(n187), .Z(n190) );
  HS65_GS_NOR2X6 U71 ( .A(n201), .B(n204), .Z(irq_acc[0]) );
  HS65_GS_AOI22X6 U72 ( .A(n442), .B(n281), .C(n449), .D(n318), .Z(n315) );
  HS65_GS_AOI33X5 U73 ( .A(n281), .B(n440), .C(n441), .D(n318), .E(n447), .F(
        n448), .Z(n316) );
  HS65_GS_OAI22X6 U74 ( .A(n318), .B(n443), .C(n450), .D(n281), .Z(n321) );
  HS65_GS_NAND2X7 U75 ( .A(n320), .B(n438), .Z(n281) );
  HS65_GS_NOR2X6 U76 ( .A(n281), .B(n443), .Z(n61) );
  HS65_GS_NAND2X7 U77 ( .A(n117), .B(n119), .Z(n317) );
  HS65_GS_NAND3X5 U78 ( .A(n125), .B(n132), .C(n297), .Z(n260) );
  HS65_GS_OAI21X3 U79 ( .A(n123), .B(n124), .C(n103), .Z(n297) );
  HS65_GS_NOR2AX3 U80 ( .A(n127), .B(n102), .Z(n259) );
  HS65_GS_AO12X9 U81 ( .A(n110), .B(n311), .C(n269), .Z(n293) );
  HS65_GS_NOR3X4 U82 ( .A(n99), .B(n81), .C(n70), .Z(n69) );
  HS65_GS_CBI4I6X5 U83 ( .A(n117), .B(n339), .C(n100), .D(n193), .Z(n99) );
  HS65_GS_OAI22X6 U84 ( .A(n446), .B(n309), .C(n295), .D(n102), .Z(n66) );
  HS65_GS_NOR3X4 U85 ( .A(n157), .B(n151), .C(n148), .Z(n164) );
  HS65_GS_NOR2X6 U86 ( .A(n446), .B(n445), .Z(n103) );
  HS65_GS_NOR2X6 U87 ( .A(n119), .B(n193), .Z(n81) );
  HS65_GS_NAND3X5 U88 ( .A(n100), .B(n103), .C(n99), .Z(n73) );
  HS65_GS_NOR2X6 U89 ( .A(n110), .B(n193), .Z(n67) );
  HS65_GSS_XNOR2X6 U90 ( .A(n66), .B(n110), .Z(n65) );
  HS65_GS_IVX9 U91 ( .A(n70), .Z(n279) );
  HS65_GS_CBI4I1X5 U92 ( .A(n311), .B(n280), .C(n269), .D(n254), .Z(n275) );
  HS65_GS_OAI21X3 U93 ( .A(n442), .B(n331), .C(n110), .Z(n280) );
  HS65_GS_OAI211X5 U94 ( .A(n439), .B(n54), .C(n296), .D(n36), .Z(n40) );
  HS65_GS_NOR3AX2 U95 ( .A(n43), .B(n438), .C(n49), .Z(n39) );
  HS65_GS_OAI21X3 U96 ( .A(n442), .B(n261), .C(n73), .Z(n71) );
  HS65_GS_NAND2X7 U97 ( .A(n298), .B(n438), .Z(n36) );
  HS65_GS_NAND2X7 U98 ( .A(n61), .B(n442), .Z(n57) );
  HS65_GS_OAI21X3 U99 ( .A(n444), .B(n57), .C(n45), .Z(n50) );
  HS65_GS_IVX9 U100 ( .A(n119), .Z(n339) );
  HS65_GS_NAND2X7 U101 ( .A(n60), .B(n436), .Z(n54) );
  HS65_GS_NAND2X7 U102 ( .A(n60), .B(n437), .Z(n49) );
  HS65_GS_NOR2X6 U103 ( .A(n102), .B(n193), .Z(n122) );
  HS65_GS_NOR2X6 U104 ( .A(n444), .B(n193), .Z(n133) );
  HS65_GS_NAND2X7 U105 ( .A(n49), .B(n44), .Z(n58) );
  HS65_GS_NOR2X6 U106 ( .A(n439), .B(n193), .Z(n48) );
  HS65_GS_NOR2X6 U107 ( .A(n54), .B(n438), .Z(n38) );
  HS65_GS_OAI21X3 U108 ( .A(n193), .B(n132), .C(n261), .Z(n41) );
  HS65_GS_IVX9 U109 ( .A(n62), .Z(n331) );
  HS65_GS_IVX9 U110 ( .A(n111), .Z(n258) );
  HS65_GS_OA112X9 U111 ( .A(n61), .B(n62), .C(n444), .D(n442), .Z(n59) );
  HS65_GS_IVX9 U112 ( .A(n44), .Z(n305) );
  HS65_GS_IVX9 U113 ( .A(n32), .Z(n198) );
  HS65_GS_IVX9 U114 ( .A(n256), .Z(n453) );
  HS65_GS_IVX9 U115 ( .A(n106), .Z(n458) );
  HS65_GS_NOR2X6 U116 ( .A(n463), .B(n247), .Z(decode_noirq) );
  HS65_GS_OA22X9 U117 ( .A(n307), .B(n459), .C(n306), .D(n463), .Z(n312) );
  HS65_GS_BFX9 U118 ( .A(n431), .Z(n187) );
  HS65_GS_BFX9 U119 ( .A(n188), .Z(n192) );
  HS65_GS_BFX9 U120 ( .A(n431), .Z(n188) );
  HS65_GS_NAND3X5 U121 ( .A(n466), .B(n465), .C(n24), .Z(n201) );
  HS65_GS_NAND2X7 U122 ( .A(n467), .B(n464), .Z(n204) );
  HS65_GS_NOR2X6 U123 ( .A(n201), .B(n203), .Z(irq_acc[1]) );
  HS65_GS_NOR2X6 U124 ( .A(n201), .B(n202), .Z(irq_acc[9]) );
  HS65_GS_NOR2X6 U125 ( .A(n28), .B(n201), .Z(irq_acc[8]) );
  HS65_GS_NOR2X6 U126 ( .A(n204), .B(n206), .Z(irq_acc[2]) );
  HS65_GS_NOR2X6 U127 ( .A(n203), .B(n206), .Z(irq_acc[3]) );
  HS65_GS_NOR2X6 U128 ( .A(n204), .B(n205), .Z(irq_acc[4]) );
  HS65_GS_NOR2X6 U129 ( .A(n203), .B(n205), .Z(irq_acc[5]) );
  HS65_GS_NOR2X6 U130 ( .A(n28), .B(n206), .Z(irq_acc[10]) );
  HS65_GS_NOR2X6 U131 ( .A(n202), .B(n206), .Z(irq_acc[11]) );
  HS65_GS_NOR2X6 U132 ( .A(n28), .B(n205), .Z(irq_acc[12]) );
  HS65_GS_NOR2X6 U133 ( .A(n202), .B(n205), .Z(irq_acc[13]) );
  HS65_GS_NOR2X6 U134 ( .A(n29), .B(n204), .Z(irq_acc[6]) );
  HS65_GS_NOR2X6 U135 ( .A(n29), .B(n203), .Z(irq_acc[7]) );
  HS65_GS_AOI12X2 U136 ( .A(n436), .B(n437), .C(OBSERVE_irq_detect), .Z(n60)
         );
  HS65_GS_NAND2X7 U137 ( .A(n60), .B(mdb_in[7]), .Z(n110) );
  HS65_GS_NAND2X7 U138 ( .A(n320), .B(mdb_in[13]), .Z(n119) );
  HS65_GS_IVX9 U139 ( .A(mdb_in[13]), .Z(n438) );
  HS65_GS_OAI211X5 U140 ( .A(n123), .B(n124), .C(n446), .D(mdb_in[5]), .Z(n125) );
  HS65_GS_AOI13X5 U141 ( .A(n61), .B(mdb_in[9]), .C(mdb_in[7]), .D(
        OBSERVE_irq_detect), .Z(n130) );
  HS65_GS_NAND3X5 U142 ( .A(mdb_in[9]), .B(n444), .C(n61), .Z(n132) );
  HS65_GS_IVX9 U143 ( .A(mdb_in[9]), .Z(n442) );
  HS65_GS_NOR3X4 U144 ( .A(mdb_in[14]), .B(mdb_in[15]), .C(OBSERVE_irq_detect), 
        .Z(n320) );
  HS65_GS_AOI32X5 U145 ( .A(e_state_nxt[0]), .B(e_state_nxt[3]), .C(
        e_state_nxt[2]), .D(n246), .E(n247), .Z(fetch) );
  HS65_GS_IVX9 U146 ( .A(mdb_in[8]), .Z(n443) );
  HS65_GS_IVX9 U147 ( .A(mdb_in[3]), .Z(n447) );
  HS65_GS_IVX9 U148 ( .A(mdb_in[2]), .Z(n448) );
  HS65_GS_IVX9 U149 ( .A(mdb_in[1]), .Z(n449) );
  HS65_GS_IVX9 U150 ( .A(mdb_in[0]), .Z(n450) );
  HS65_GS_IVX9 U151 ( .A(mdb_in[10]), .Z(n441) );
  HS65_GS_IVX9 U152 ( .A(mdb_in[11]), .Z(n440) );
  HS65_GS_NAND3AX6 U153 ( .A(n259), .B(n430), .C(n262), .Z(n269) );
  HS65_GS_AO112X9 U154 ( .A(n480), .B(n249), .C(n250), .D(n251), .Z(
        e_state_nxt[3]) );
  HS65_GS_AO311X9 U155 ( .A(n252), .B(n253), .C(n254), .D(n255), .E(n142), .Z(
        n251) );
  HS65_GS_AOI12X2 U156 ( .A(n256), .B(n257), .C(n471), .Z(n255) );
  HS65_GS_OAI211X5 U157 ( .A(n259), .B(n260), .C(n430), .D(n262), .Z(n252) );
  HS65_GS_OR2X9 U158 ( .A(n130), .B(cpu_halt_st), .Z(n254) );
  HS65_GS_OAI22X6 U159 ( .A(n214), .B(n242), .C(n228), .D(n233), .Z(
        inst_dest[15]) );
  HS65_GS_OAI22X6 U160 ( .A(n214), .B(n227), .C(n228), .D(n238), .Z(
        inst_dest[11]) );
  HS65_GS_OAI22X6 U161 ( .A(n208), .B(n242), .C(n228), .D(n235), .Z(
        inst_dest[13]) );
  HS65_GS_OAI22X6 U162 ( .A(n211), .B(n242), .C(n230), .D(n235), .Z(
        inst_dest[12]) );
  HS65_GS_OAI22X6 U163 ( .A(n217), .B(n227), .C(n230), .D(n238), .Z(
        inst_dest[10]) );
  HS65_GS_OAI22X6 U164 ( .A(n217), .B(n242), .C(n230), .D(n233), .Z(
        inst_dest[14]) );
  HS65_GS_OAI222X2 U165 ( .A(n211), .B(n237), .C(n229), .D(n234), .E(
        cpu_halt_st), .F(n475), .Z(inst_dest[0]) );
  HS65_GS_OAI22X6 U166 ( .A(n211), .B(n231), .C(n234), .D(n235), .Z(
        inst_dest[4]) );
  HS65_GS_OAI212X5 U167 ( .A(n229), .B(n232), .C(n208), .D(n237), .E(n239), 
        .Z(inst_dest[1]) );
  HS65_GS_NAND3AX6 U168 ( .A(cpu_halt_st), .B(n475), .C(n240), .Z(n239) );
  HS65_GS_OAI22X6 U169 ( .A(n214), .B(n237), .C(n232), .D(n238), .Z(
        inst_dest[3]) );
  HS65_GS_OAI22X6 U170 ( .A(n217), .B(n237), .C(n234), .D(n238), .Z(
        inst_dest[2]) );
  HS65_GS_OAI22X6 U171 ( .A(n217), .B(n223), .C(n212), .D(n216), .Z(
        inst_src[14]) );
  HS65_GS_OAI22X6 U172 ( .A(n208), .B(n223), .C(n209), .D(n219), .Z(
        inst_src[13]) );
  HS65_GS_OAI22X6 U173 ( .A(n207), .B(n217), .C(n212), .D(n222), .Z(
        inst_src[10]) );
  HS65_GS_OAI22X6 U174 ( .A(n211), .B(n223), .C(n212), .D(n219), .Z(
        inst_src[12]) );
  HS65_GS_OAI22X6 U175 ( .A(n207), .B(n214), .C(n209), .D(n222), .Z(
        inst_src[11]) );
  HS65_GS_NOR3X4 U176 ( .A(n99), .B(n104), .C(n456), .Z(n70) );
  HS65_GS_OAI22X6 U177 ( .A(n214), .B(n221), .C(n215), .D(n222), .Z(
        inst_src[3]) );
  HS65_GS_OAI32X5 U178 ( .A(n456), .B(pc_sw_wr), .C(n310), .D(
        OBSERVE_irq_detect), .E(n312), .Z(i_state_nxt[2]) );
  HS65_GS_NAND2X7 U179 ( .A(n488), .B(n489), .Z(n211) );
  HS65_GS_NOR2X6 U180 ( .A(n166), .B(n173), .Z(n174) );
  HS65_GS_OAI22X6 U181 ( .A(n208), .B(n213), .C(n215), .D(n219), .Z(
        inst_src[5]) );
  HS65_GS_OAI22X6 U182 ( .A(n211), .B(n213), .C(n218), .D(n219), .Z(
        inst_src[4]) );
  HS65_GS_IVX9 U183 ( .A(mdb_in[7]), .Z(n444) );
  HS65_GS_NOR3X4 U184 ( .A(n167), .B(n169), .C(n151), .Z(n171) );
  HS65_GS_OAI32X5 U185 ( .A(n331), .B(n442), .C(n258), .D(n195), .E(n469), .Z(
        n406) );
  HS65_GS_OAI212X5 U186 ( .A(n460), .B(n302), .C(OBSERVE_irq_detect), .D(n303), 
        .E(n461), .Z(i_state_nxt[1]) );
  HS65_GS_IVX9 U187 ( .A(n291), .Z(n461) );
  HS65_GS_AOI22X6 U188 ( .A(n306), .B(n246), .C(n307), .D(n308), .Z(n303) );
  HS65_GS_OAI31X5 U189 ( .A(n474), .B(n46), .C(n310), .D(n473), .Z(n302) );
  HS65_GS_OAI22X6 U190 ( .A(n207), .B(n211), .C(n210), .D(n212), .Z(
        inst_src[8]) );
  HS65_GS_OAI32X5 U191 ( .A(n152), .B(n153), .C(n161), .D(OBSERVE_irq_detect), 
        .E(n467), .Z(n416) );
  HS65_GS_NAND2X7 U192 ( .A(mdb_in[4]), .B(n445), .Z(n102) );
  HS65_GS_OAI32X5 U193 ( .A(n156), .B(n157), .C(n152), .D(OBSERVE_irq_detect), 
        .E(n466), .Z(n415) );
  HS65_GS_NAND2X7 U194 ( .A(n159), .B(n432), .Z(n156) );
  HS65_GS_IVX9 U195 ( .A(mdb_in[4]), .Z(n446) );
  HS65_GS_NAND3X5 U196 ( .A(mdb_in[14]), .B(mdb_in[15]), .C(n60), .Z(n44) );
  HS65_GS_NAND3X5 U197 ( .A(mdb_in[13]), .B(n58), .C(mdb_in[12]), .Z(n45) );
  HS65_GS_OAI22X6 U198 ( .A(n213), .B(n217), .C(n216), .D(n218), .Z(
        inst_src[6]) );
  HS65_GS_IVX9 U199 ( .A(mdb_in[5]), .Z(n445) );
  HS65_GS_FA1X4 U200 ( .A0(mdb_in[2]), .B0(N702), .CI(add_477_carry[2]), .CO(
        add_477_carry[3]), .S0(ext_nxt_2_) );
  HS65_GS_NAND2X7 U201 ( .A(n176), .B(n168), .Z(n151) );
  HS65_GS_NOR2X6 U202 ( .A(n193), .B(mdb_in[7]), .Z(n111) );
  HS65_GS_NAND3X5 U203 ( .A(n487), .B(n486), .C(n236), .Z(n237) );
  HS65_GS_OAI22X6 U204 ( .A(n213), .B(n214), .C(n215), .D(n216), .Z(
        inst_src[7]) );
  HS65_GS_NAND3X5 U205 ( .A(n487), .B(n486), .C(n220), .Z(n221) );
  HS65_GS_OAI21X3 U206 ( .A(n160), .B(n163), .C(OBSERVE_irq_detect), .Z(n152)
         );
  HS65_GS_NOR2X6 U207 ( .A(n281), .B(mdb_in[8]), .Z(n62) );
  HS65_GS_NOR3AX2 U208 ( .A(n43), .B(n44), .C(mdb_in[13]), .Z(n33) );
  HS65_GS_NAND3X5 U209 ( .A(mdb_in[5]), .B(n446), .C(n118), .Z(n96) );
  HS65_GS_IVX9 U210 ( .A(mdb_in[14]), .Z(n437) );
  HS65_GS_IVX9 U211 ( .A(mdb_in[15]), .Z(n436) );
  HS65_GS_NOR2X6 U212 ( .A(n193), .B(mdb_in[12]), .Z(n43) );
  HS65_GS_OAI22X6 U213 ( .A(n195), .B(n487), .C(n194), .D(n448), .Z(n422) );
  HS65_GS_OAI22X6 U214 ( .A(n195), .B(n486), .C(n194), .D(n447), .Z(n421) );
  HS65_GS_OAI22X6 U215 ( .A(n195), .B(n477), .C(n194), .D(n132), .Z(n404) );
  HS65_GS_OAI22X6 U216 ( .A(n195), .B(n491), .C(n193), .D(n441), .Z(n427) );
  HS65_GS_OAI22X6 U217 ( .A(n195), .B(n490), .C(n193), .D(n440), .Z(n426) );
  HS65_GS_OAI22X6 U218 ( .A(OBSERVE_irq_detect), .B(n465), .C(n151), .D(n149), 
        .Z(n414) );
  HS65_GS_OAI22X6 U219 ( .A(OBSERVE_irq_detect), .B(n464), .C(n148), .D(n149), 
        .Z(n413) );
  HS65_GS_NOR3X4 U220 ( .A(n226), .B(n490), .C(n491), .Z(inst_jmp[3]) );
  HS65_GS_OAI22X6 U221 ( .A(n195), .B(n485), .C(n193), .D(n443), .Z(n420) );
  HS65_GS_OAI22X6 U222 ( .A(n195), .B(n483), .C(n194), .D(n440), .Z(n417) );
  HS65_GS_OAI22X6 U223 ( .A(n207), .B(n208), .C(n209), .D(n210), .Z(
        inst_src[9]) );
  HS65_GS_NAND3X5 U224 ( .A(n494), .B(n497), .C(cpu_halt_st), .Z(n234) );
  HS65_GS_OAI22X6 U225 ( .A(n195), .B(n476), .C(n194), .D(n130), .Z(n403) );
  HS65_GS_OAI22X6 U226 ( .A(n195), .B(n470), .C(n194), .D(n442), .Z(n419) );
  HS65_GS_OAI22X6 U227 ( .A(n195), .B(n488), .C(n193), .D(n449), .Z(n423) );
  HS65_GS_OAI22X6 U228 ( .A(OBSERVE_decode), .B(n489), .C(n194), .D(n450), .Z(
        n424) );
  HS65_GS_OAI22X6 U229 ( .A(OBSERVE_decode), .B(n484), .C(n194), .D(n441), .Z(
        n418) );
  HS65_GS_NAND2X7 U230 ( .A(n470), .B(n484), .Z(n210) );
  HS65_GS_OA311X9 U231 ( .A(n117), .B(mdb_in[5]), .C(n339), .D(n96), .E(n93), 
        .Z(n100) );
  HS65_GS_CBI4I1X5 U232 ( .A(n312), .B(n313), .C(OBSERVE_irq_detect), .D(n462), 
        .Z(i_state_nxt[0]) );
  HS65_GS_OAI211X5 U233 ( .A(n65), .B(n66), .C(n451), .D(decode_noirq), .Z(
        n313) );
  HS65_GS_NAND4ABX3 U234 ( .A(n153), .B(n161), .C(n154), .D(n164), .Z(n163) );
  HS65_GS_NAND4ABX3 U235 ( .A(n166), .B(n167), .C(n168), .D(n159), .Z(n161) );
  HS65_GS_NAND4ABX3 U236 ( .A(n434), .B(n173), .C(n170), .D(n168), .Z(n157) );
  HS65_GS_IVX9 U237 ( .A(n179), .Z(n434) );
  HS65_GS_FA1X4 U238 ( .A0(mdb_in[13]), .B0(N702), .CI(add_477_carry[13]), 
        .CO(add_477_carry[14]), .S0(ext_nxt_13_) );
  HS65_GS_FA1X4 U239 ( .A0(mdb_in[9]), .B0(N702), .CI(add_477_carry[9]), .CO(
        add_477_carry[10]), .S0(ext_nxt_9_) );
  HS65_GS_FA1X4 U240 ( .A0(mdb_in[7]), .B0(N702), .CI(add_477_carry[7]), .CO(
        add_477_carry[8]), .S0(ext_nxt_7_) );
  HS65_GS_FA1X4 U241 ( .A0(mdb_in[5]), .B0(N702), .CI(add_477_carry[5]), .CO(
        add_477_carry[6]), .S0(ext_nxt_5_) );
  HS65_GS_FA1X4 U242 ( .A0(mdb_in[12]), .B0(N702), .CI(add_477_carry[12]), 
        .CO(add_477_carry[13]), .S0(ext_nxt_12_) );
  HS65_GS_FA1X4 U243 ( .A0(mdb_in[14]), .B0(N702), .CI(add_477_carry[14]), 
        .CO(add_477_carry[15]), .S0(ext_nxt_14_) );
  HS65_GS_FA1X4 U244 ( .A0(mdb_in[4]), .B0(N702), .CI(add_477_carry[4]), .CO(
        add_477_carry[5]), .S0(ext_nxt_4_) );
  HS65_GS_FA1X4 U245 ( .A0(mdb_in[8]), .B0(N702), .CI(add_477_carry[8]), .CO(
        add_477_carry[9]), .S0(ext_nxt_8_) );
  HS65_GS_FA1X4 U246 ( .A0(mdb_in[3]), .B0(N702), .CI(add_477_carry[3]), .CO(
        add_477_carry[4]), .S0(ext_nxt_3_) );
  HS65_GS_FA1X4 U247 ( .A0(mdb_in[10]), .B0(N702), .CI(add_477_carry[10]), 
        .CO(add_477_carry[11]), .S0(ext_nxt_10_) );
  HS65_GS_FA1X4 U248 ( .A0(mdb_in[11]), .B0(N702), .CI(add_477_carry[11]), 
        .CO(add_477_carry[12]), .S0(ext_nxt_11_) );
  HS65_GS_FA1X4 U249 ( .A0(mdb_in[6]), .B0(N702), .CI(add_477_carry[6]), .CO(
        add_477_carry[7]), .S0(ext_nxt_6_) );
  HS65_GS_NAND4ABX3 U250 ( .A(n169), .B(n167), .C(n159), .D(n170), .Z(n148) );
  HS65_GSS_XOR3X2 U251 ( .A(mdb_in[15]), .B(N702), .C(add_477_carry[15]), .Z(
        ext_nxt_15_) );
  HS65_GS_NAND4ABX3 U252 ( .A(n152), .B(n153), .C(n154), .D(n432), .Z(n149) );
  HS65_GS_OAI22X6 U253 ( .A(n214), .B(n223), .C(n209), .D(n216), .Z(
        inst_src[15]) );
  HS65_GS_OAI22X6 U254 ( .A(n208), .B(n227), .C(n228), .D(n229), .Z(
        inst_dest[9]) );
  HS65_GS_OAI22X6 U255 ( .A(n217), .B(n231), .C(n233), .D(n234), .Z(
        inst_dest[6]) );
  HS65_GS_OAI22X6 U256 ( .A(n217), .B(n221), .C(n218), .D(n222), .Z(
        inst_src[2]) );
  HS65_GS_OAI22X6 U257 ( .A(n214), .B(n231), .C(n232), .D(n233), .Z(
        inst_dest[7]) );
  HS65_GS_OAI22X6 U258 ( .A(n211), .B(n227), .C(n229), .D(n230), .Z(
        inst_dest[8]) );
  HS65_GS_OAI22X6 U259 ( .A(n208), .B(n231), .C(n232), .D(n235), .Z(
        inst_dest[5]) );
  HS65_GS_NOR2X6 U260 ( .A(OBSERVE_irq_detect), .B(decode_noirq), .Z(n32) );
  HS65_GS_NOR3AX2 U261 ( .A(i_state_nxt[0]), .B(n289), .C(n114), .Z(N249) );
  HS65_GS_IVX9 U262 ( .A(i_state_nxt[2]), .Z(n289) );
  HS65_GS_IVX9 U263 ( .A(n160), .Z(n432) );
  HS65_GS_OAI21X3 U264 ( .A(n195), .B(n475), .C(n261), .Z(n402) );
  HS65_GS_AND3X9 U265 ( .A(n177), .B(n178), .C(n179), .Z(n176) );
  HS65_GS_NOR3X4 U266 ( .A(n454), .B(n468), .C(n479), .Z(n145) );
  HS65_GS_NOR4ABX2 U267 ( .A(n450), .B(n449), .C(mdb_in[3]), .D(mdb_in[2]), 
        .Z(n108) );
  HS65_GS_NOR4ABX2 U268 ( .A(n447), .B(mdb_in[1]), .C(mdb_in[2]), .D(mdb_in[0]), .Z(n68) );
  HS65_GS_NAND2X7 U269 ( .A(n330), .B(n454), .Z(n256) );
  HS65_GS_IVX9 U270 ( .A(mdb_in[12]), .Z(n439) );
  HS65_GS_IVX9 U271 ( .A(n143), .Z(n479) );
  HS65_GS_AOI12X6 U272 ( .A(n290), .B(n104), .C(n291), .Z(n106) );
  HS65_GS_IVX9 U273 ( .A(n290), .Z(n456) );
  HS65_GS_NOR3X4 U274 ( .A(n24), .B(pc_sw_wr), .C(n25), .Z(n9) );
  HS65_GS_IVX9 U275 ( .A(n24), .Z(n462) );
  HS65_GS_AND2X4 U276 ( .A(n25), .B(n451), .Z(n12) );
  HS65_GS_IVX9 U277 ( .A(pc_sw_wr), .Z(n451) );
  HS65_GS_OAI21X3 U278 ( .A(n257), .B(n481), .C(n199), .Z(n250) );
  HS65_GS_NOR2X6 U279 ( .A(n462), .B(pc_sw_wr), .Z(n10) );
  HS65_GS_IVX9 U280 ( .A(exec_done), .Z(n452) );
  HS65_GS_IVX9 U281 ( .A(n199), .Z(n455) );
  HS65_GS_NAND2X7 U282 ( .A(n270), .B(n256), .Z(n253) );
  HS65_GS_OA31X9 U283 ( .A(n454), .B(n104), .C(n456), .D(n268), .Z(n287) );
  HS65_GS_OA22X9 U284 ( .A(n114), .B(n299), .C(n307), .D(n452), .Z(n306) );
  HS65_GS_AND2X4 U285 ( .A(n114), .B(n459), .Z(n262) );
  HS65_GS_IVX9 U286 ( .A(n308), .Z(n459) );
  HS65_GS_AND2X4 U287 ( .A(n299), .B(n452), .Z(n247) );
  HS65_GS_OAI21X3 U288 ( .A(n453), .B(n481), .C(n199), .Z(n429) );
  HS65_GS_IVX9 U289 ( .A(n246), .Z(n463) );
  HS65_GS_NOR2X6 U290 ( .A(n455), .B(n121), .Z(n394) );
  HS65_GS_IVX9 U291 ( .A(puc_rst), .Z(n431) );
  HS65_GS_NOR3X4 U292 ( .A(n225), .B(n490), .C(n491), .Z(inst_jmp[7]) );
  HS65_GS_BFX9 U293 ( .A(mdb_in[13]), .Z(OBSERVE_ir[13]) );
  HS65_GS_BFX9 U294 ( .A(mdb_in[9]), .Z(OBSERVE_ir[9]) );
  HS65_GS_BFX9 U295 ( .A(mdb_in[7]), .Z(OBSERVE_ir[7]) );
  HS65_GS_BFX9 U296 ( .A(mdb_in[5]), .Z(OBSERVE_ir[5]) );
  HS65_GS_BFX9 U297 ( .A(mdb_in[12]), .Z(OBSERVE_ir[12]) );
  HS65_GS_AND2X4 U298 ( .A(n114), .B(n430), .Z(n307) );
  HS65_GS_BFX9 U299 ( .A(mdb_in[14]), .Z(OBSERVE_ir[14]) );
  HS65_GS_BFX9 U300 ( .A(mdb_in[15]), .Z(OBSERVE_ir[15]) );
  HS65_GS_BFX9 U301 ( .A(mdb_in[2]), .Z(OBSERVE_ir[2]) );
  HS65_GS_BFX9 U302 ( .A(mdb_in[4]), .Z(OBSERVE_ir[4]) );
  HS65_GS_BFX9 U303 ( .A(mdb_in[0]), .Z(OBSERVE_ir[0]) );
  HS65_GS_BFX9 U304 ( .A(mdb_in[8]), .Z(OBSERVE_ir[8]) );
  HS65_GS_BFX9 U305 ( .A(mdb_in[3]), .Z(OBSERVE_ir[3]) );
  HS65_GS_BFX9 U306 ( .A(mdb_in[1]), .Z(OBSERVE_ir[1]) );
  HS65_GS_BFX9 U307 ( .A(mdb_in[10]), .Z(OBSERVE_ir[10]) );
  HS65_GS_BFX9 U308 ( .A(mdb_in[11]), .Z(OBSERVE_ir[11]) );
  HS65_GS_BFX9 U309 ( .A(mdb_in[6]), .Z(OBSERVE_ir[6]) );
  HS65_GS_NOR2X6 U310 ( .A(n28), .B(n29), .Z(nmi_acc) );
  HS65_GS_BFX9 U311 ( .A(pc_nxt[0]), .Z(mab[0]) );
  HS65_GS_NAND2X7 U312 ( .A(n495), .B(n496), .Z(n229) );
  HS65_GS_NOR3X4 U313 ( .A(OBSERVE_i_state[1]), .B(OBSERVE_i_state[2]), .C(
        OBSERVE_i_state[0]), .Z(n24) );
  HS65_GS_NAND3X5 U314 ( .A(n24), .B(n465), .C(OBSERVE_irq_num[1]), .Z(n206)
         );
  HS65_GS_NAND3X5 U315 ( .A(n24), .B(n466), .C(OBSERVE_irq_num[2]), .Z(n205)
         );
  HS65_GS_NAND3X5 U316 ( .A(OBSERVE_irq_num[2]), .B(n24), .C(
        OBSERVE_irq_num[1]), .Z(n29) );
  HS65_GS_NAND2X7 U317 ( .A(OBSERVE_irq_num[0]), .B(n464), .Z(n203) );
  HS65_GS_NAND2X7 U318 ( .A(OBSERVE_irq_num[3]), .B(n467), .Z(n28) );
  HS65_GS_NAND2X7 U319 ( .A(OBSERVE_irq_num[0]), .B(OBSERVE_irq_num[3]), .Z(
        n202) );
  HS65_GS_IVX9 U320 ( .A(OBSERVE_irq_num[1]), .Z(n466) );
  HS65_GS_IVX9 U321 ( .A(OBSERVE_irq_num[2]), .Z(n465) );
  HS65_GS_IVX9 U322 ( .A(OBSERVE_irq_num[0]), .Z(n467) );
  HS65_GS_IVX9 U323 ( .A(OBSERVE_irq_num[3]), .Z(n464) );
  HS65_GS_AO112X9 U324 ( .A(pc_incr_15_), .B(n9), .C(n10), .D(n17), .Z(mab[15]) );
  HS65_GS_AO22X9 U325 ( .A(pc_sw[15]), .B(pc_sw_wr), .C(mdb_in[15]), .D(n12), 
        .Z(n17) );
  HS65_GS_NAND4ABX3 U326 ( .A(pmem_busy), .B(fetch), .C(n451), .D(n200), .Z(
        mb_en) );
  HS65_GS_AOI12X2 U327 ( .A(cpu_halt_st), .B(n114), .C(n24), .Z(n200) );
  HS65_GS_OAI311X5 U328 ( .A(n257), .B(exec_dst_wr), .C(n263), .D(n264), .E(
        n265), .Z(e_state_nxt[2]) );
  HS65_GS_AOI22X6 U329 ( .A(e_state[0]), .B(n266), .C(exec_jmp), .D(n453), .Z(
        n265) );
  HS65_GS_OAI211X5 U330 ( .A(n260), .B(n269), .C(n254), .D(n253), .Z(n264) );
  HS65_GS_AO31X9 U331 ( .A(e_state[1]), .B(n482), .C(n480), .D(n268), .Z(n266)
         );
  HS65_GS_OA112X9 U332 ( .A(exec_done), .B(n308), .C(n114), .D(n322), .Z(
        OBSERVE_irq_detect) );
  HS65_GS_AOI12X2 U333 ( .A(n323), .B(n435), .C(cpu_halt_st), .Z(n322) );
  HS65_GS_OAI21X3 U334 ( .A(n324), .B(n325), .C(gie), .Z(n323) );
  HS65_GS_NAND4ABX3 U335 ( .A(irq[0]), .B(irq[10]), .C(n326), .D(n183), .Z(
        n325) );
  HS65_GS_AO112X9 U336 ( .A(pc_incr_14_), .B(n9), .C(n10), .D(n18), .Z(mab[14]) );
  HS65_GS_AO22X9 U337 ( .A(pc_sw[14]), .B(pc_sw_wr), .C(mdb_in[14]), .D(n12), 
        .Z(n18) );
  HS65_GS_NOR2X6 U338 ( .A(irq[13]), .B(irq[12]), .Z(n183) );
  HS65_GS_NAND4ABX3 U339 ( .A(n277), .B(n455), .C(n284), .D(n285), .Z(
        e_state_nxt[0]) );
  HS65_GS_AOI312X4 U340 ( .A(n457), .B(n480), .C(n249), .D(n271), .E(n481), 
        .F(n287), .Z(n285) );
  HS65_GS_AOI33X5 U341 ( .A(n142), .B(n337), .C(n292), .D(n276), .E(n254), .F(
        n293), .Z(n284) );
  HS65_GS_IVX9 U342 ( .A(n121), .Z(n457) );
  HS65_GS_AO22X9 U343 ( .A(pc_sw[12]), .B(pc_sw_wr), .C(mdb_in[12]), .D(n12), 
        .Z(n20) );
  HS65_GS_AO22X9 U344 ( .A(pc_sw[11]), .B(pc_sw_wr), .C(mdb_in[11]), .D(n12), 
        .Z(n21) );
  HS65_GS_AO22X9 U345 ( .A(pc_sw[10]), .B(pc_sw_wr), .C(mdb_in[10]), .D(n12), 
        .Z(n22) );
  HS65_GS_AO22X9 U346 ( .A(pc_sw_wr), .B(pc_sw[9]), .C(mdb_in[9]), .D(n12), 
        .Z(n11) );
  HS65_GS_AO22X9 U347 ( .A(pc_sw[8]), .B(pc_sw_wr), .C(mdb_in[8]), .D(n12), 
        .Z(n13) );
  HS65_GS_AO22X9 U348 ( .A(pc_sw[7]), .B(pc_sw_wr), .C(mdb_in[7]), .D(n12), 
        .Z(n14) );
  HS65_GS_AO22X9 U349 ( .A(pc_sw[6]), .B(pc_sw_wr), .C(mdb_in[6]), .D(n12), 
        .Z(n15) );
  HS65_GS_AO22X9 U350 ( .A(pc_sw[5]), .B(pc_sw_wr), .C(mdb_in[5]), .D(n12), 
        .Z(n16) );
  HS65_GS_BFX9 U351 ( .A(mab[15]), .Z(pc_nxt[15]) );
  HS65_GS_BFX9 U352 ( .A(mab[14]), .Z(pc_nxt[14]) );
  HS65_GS_AO22X9 U353 ( .A(pc_sw[13]), .B(pc_sw_wr), .C(mdb_in[13]), .D(n12), 
        .Z(n19) );
  HS65_GS_OR4X4 U354 ( .A(irq[6]), .B(n327), .C(irq[4]), .D(irq[5]), .Z(n324)
         );
  HS65_GS_OR4X4 U355 ( .A(wdt_irq), .B(irq[9]), .C(irq[8]), .D(irq[7]), .Z(
        n327) );
  HS65_GS_NOR4X4 U356 ( .A(irq[11]), .B(irq[1]), .C(irq[3]), .D(irq[2]), .Z(
        n326) );
  HS65_GS_NOR3X4 U357 ( .A(cpu_halt_st), .B(inst_type[1]), .C(n240), .Z(n236)
         );
  HS65_GS_NOR4ABX2 U358 ( .A(n477), .B(inst_type[0]), .C(inst_so[7]), .D(
        inst_type[2]), .Z(n220) );
  HS65_GS_OAI222X2 U359 ( .A(n208), .B(n221), .C(n210), .D(n215), .E(
        inst_type[2]), .F(n477), .Z(inst_src[1]) );
  HS65_GS_CBI4I1X5 U360 ( .A(n290), .B(i_state_nxt[1]), .C(n291), .D(
        inst_ad[4]), .Z(n301) );
  HS65_GS_NAND2X7 U361 ( .A(inst_dest_bin[0]), .B(n488), .Z(n208) );
  HS65_GS_NAND2X7 U362 ( .A(inst_dest_bin[0]), .B(inst_dest_bin[1]), .Z(n214)
         );
  HS65_GS_NAND2X7 U363 ( .A(inst_dest_bin[1]), .B(n489), .Z(n217) );
  HS65_GS_NOR3AX2 U364 ( .A(n174), .B(n499), .C(n151), .Z(n169) );
  HS65_GS_IVX9 U365 ( .A(irq[7]), .Z(n499) );
  HS65_GS_IVX9 U366 ( .A(inst_dest_bin[2]), .Z(n487) );
  HS65_GS_IVX9 U367 ( .A(inst_dest_bin[3]), .Z(n486) );
  HS65_GS_OAI32X5 U368 ( .A(n288), .B(n68), .C(n108), .D(n337), .E(n195), .Z(
        n388) );
  HS65_GS_IVX9 U369 ( .A(n67), .Z(n288) );
  HS65_GS_NOR4ABX2 U370 ( .A(irq[2]), .B(n154), .C(n433), .D(n161), .Z(n153)
         );
  HS65_GS_IVX9 U371 ( .A(n164), .Z(n433) );
  HS65_GS_NOR4ABX2 U372 ( .A(irq[6]), .B(n174), .C(n151), .D(n169), .Z(n167)
         );
  HS65_GS_OAI211X5 U373 ( .A(irq[13]), .B(n498), .C(n435), .D(n177), .Z(n166)
         );
  HS65_GS_IVX9 U374 ( .A(irq[12]), .Z(n498) );
  HS65_GS_IVX9 U375 ( .A(inst_jmp_bin[0]), .Z(n491) );
  HS65_GS_IVX9 U376 ( .A(inst_jmp_bin[1]), .Z(n490) );
  HS65_GS_NAND3X5 U377 ( .A(inst_dest_bin[3]), .B(n487), .C(n220), .Z(n207) );
  HS65_GS_NAND2X7 U378 ( .A(inst_jmp_bin[2]), .B(inst_type[1]), .Z(n225) );
  HS65_GS_NAND3X5 U379 ( .A(inst_dest_bin[2]), .B(n486), .C(n220), .Z(n213) );
  HS65_GS_OAI212X5 U380 ( .A(n278), .B(n96), .C(n449), .D(n261), .E(n98), .Z(
        n368) );
  HS65_GS_AOI212X4 U381 ( .A(ext_nxt_2_), .B(n70), .C(inst_sext[2]), .D(n69), 
        .E(n286), .Z(n98) );
  HS65_GS_NAND3X5 U382 ( .A(inst_dest_bin[2]), .B(n486), .C(n236), .Z(n231) );
  HS65_GS_NAND3X5 U383 ( .A(inst_dest_bin[3]), .B(n487), .C(n236), .Z(n227) );
  HS65_GS_NAND3X5 U384 ( .A(inst_dest_bin[2]), .B(inst_dest_bin[3]), .C(n236), 
        .Z(n242) );
  HS65_GS_NAND3X5 U385 ( .A(inst_dest_bin[2]), .B(inst_dest_bin[3]), .C(n220), 
        .Z(n223) );
  HS65_GS_OAI212X5 U386 ( .A(n279), .B(n248), .C(n447), .D(n261), .E(n91), .Z(
        n366) );
  HS65_GS_IVX9 U387 ( .A(ext_nxt_4_), .Z(n248) );
  HS65_GS_AOI12X2 U388 ( .A(inst_sext[4]), .B(n69), .C(n286), .Z(n91) );
  HS65_GS_OAI212X5 U389 ( .A(n279), .B(n245), .C(n446), .D(n261), .E(n88), .Z(
        n365) );
  HS65_GS_IVX9 U390 ( .A(ext_nxt_5_), .Z(n245) );
  HS65_GS_AOI12X2 U391 ( .A(inst_sext[5]), .B(n69), .C(n286), .Z(n88) );
  HS65_GS_OAI212X5 U392 ( .A(n279), .B(n244), .C(n445), .D(n261), .E(n85), .Z(
        n364) );
  HS65_GS_IVX9 U393 ( .A(ext_nxt_6_), .Z(n244) );
  HS65_GS_AOI12X2 U394 ( .A(inst_sext[6]), .B(n69), .C(n286), .Z(n85) );
  HS65_GS_OAI212X5 U395 ( .A(n279), .B(n243), .C(n444), .D(n261), .E(n80), .Z(
        n362) );
  HS65_GS_IVX9 U396 ( .A(ext_nxt_8_), .Z(n243) );
  HS65_GS_AOI12X2 U397 ( .A(inst_sext[8]), .B(n69), .C(n286), .Z(n80) );
  HS65_GS_OAI212X5 U398 ( .A(n279), .B(n241), .C(n443), .D(n261), .E(n77), .Z(
        n361) );
  HS65_GS_AOI12X2 U399 ( .A(inst_sext[9]), .B(n69), .C(n286), .Z(n77) );
  HS65_GS_IVX9 U400 ( .A(ext_nxt_9_), .Z(n241) );
  HS65_GS_OAI212X5 U401 ( .A(n278), .B(n93), .C(n448), .D(n261), .E(n95), .Z(
        n367) );
  HS65_GS_AOI212X4 U402 ( .A(ext_nxt_3_), .B(n70), .C(inst_sext[3]), .D(n69), 
        .E(n286), .Z(n95) );
  HS65_GS_NAND3X5 U403 ( .A(inst_src_bin[3]), .B(inst_type[2]), .C(
        inst_src_bin[0]), .Z(n209) );
  HS65_GS_NOR2AX3 U404 ( .A(irq[1]), .B(n163), .Z(n160) );
  HS65_GS_NAND3X5 U405 ( .A(inst_type[2]), .B(n485), .C(inst_src_bin[3]), .Z(
        n212) );
  HS65_GS_NAND3X5 U406 ( .A(inst_type[2]), .B(n483), .C(inst_src_bin[0]), .Z(
        n215) );
  HS65_GS_NOR3X4 U407 ( .A(n226), .B(inst_jmp_bin[0]), .C(n490), .Z(
        inst_jmp[2]) );
  HS65_GS_IVX9 U408 ( .A(inst_so[6]), .Z(n477) );
  HS65_GS_NAND3AX6 U409 ( .A(n166), .B(irq[3]), .C(n164), .Z(n154) );
  HS65_GS_NOR3X4 U410 ( .A(n226), .B(inst_jmp_bin[1]), .C(n491), .Z(
        inst_jmp[1]) );
  HS65_GS_NAND2AX7 U411 ( .A(inst_jmp_bin[2]), .B(inst_type[1]), .Z(n226) );
  HS65_GS_NAND4ABX3 U412 ( .A(n166), .B(n157), .C(n171), .D(irq[4]), .Z(n159)
         );
  HS65_GS_NOR3X4 U413 ( .A(n226), .B(inst_jmp_bin[1]), .C(inst_jmp_bin[0]), 
        .Z(inst_jmp[0]) );
  HS65_GS_NOR3X4 U414 ( .A(n225), .B(inst_jmp_bin[0]), .C(n490), .Z(
        inst_jmp[6]) );
  HS65_GS_NAND3X5 U415 ( .A(n176), .B(n174), .C(irq[8]), .Z(n168) );
  HS65_GS_NAND2X7 U416 ( .A(inst_src_bin[1]), .B(inst_src_bin[2]), .Z(n216) );
  HS65_GS_NOR3X4 U417 ( .A(n225), .B(inst_jmp_bin[1]), .C(n491), .Z(
        inst_jmp[5]) );
  HS65_GS_NAND3X5 U418 ( .A(n485), .B(n483), .C(inst_type[2]), .Z(n218) );
  HS65_GS_IVX9 U419 ( .A(inst_type[1]), .Z(n475) );
  HS65_GS_NAND2X7 U420 ( .A(inst_src_bin[1]), .B(n484), .Z(n222) );
  HS65_GS_NAND2X7 U421 ( .A(inst_src_bin[2]), .B(n470), .Z(n219) );
  HS65_GS_OAI22X6 U422 ( .A(n332), .B(n195), .C(n57), .D(n258), .Z(n408) );
  HS65_GS_OAI22X6 U423 ( .A(n333), .B(n195), .C(n57), .D(n267), .Z(n407) );
  HS65_GS_IVX9 U424 ( .A(n133), .Z(n267) );
  HS65_GS_OAI22X6 U425 ( .A(n334), .B(n195), .C(n194), .D(n125), .Z(n399) );
  HS65_GS_OAI22X6 U426 ( .A(n493), .B(n195), .C(n36), .D(n194), .Z(n341) );
  HS65_GS_IVX9 U427 ( .A(inst_alu[1]), .Z(n493) );
  HS65_GS_OAI22X6 U428 ( .A(n492), .B(n195), .C(n52), .D(n194), .Z(n349) );
  HS65_GS_IVX9 U429 ( .A(inst_alu[9]), .Z(n492) );
  HS65_GS_NOR3AX2 U430 ( .A(n53), .B(n50), .C(n40), .Z(n52) );
  HS65_GS_AOI12X2 U431 ( .A(mdb_in[13]), .B(n58), .C(n59), .Z(n53) );
  HS65_GS_IVX9 U432 ( .A(inst_src_bin[0]), .Z(n485) );
  HS65_GS_IVX9 U433 ( .A(inst_src_bin[3]), .Z(n483) );
  HS65_GS_NAND3X5 U434 ( .A(n171), .B(n174), .C(irq[5]), .Z(n170) );
  HS65_GS_NOR2X6 U435 ( .A(n183), .B(nmi_pnd), .Z(n173) );
  HS65_GS_IVX9 U436 ( .A(inst_so[4]), .Z(n469) );
  HS65_GS_OAI212X5 U437 ( .A(n210), .B(n218), .C(n211), .D(n221), .E(n224), 
        .Z(inst_src[0]) );
  HS65_GS_OR3X9 U438 ( .A(inst_type[2]), .B(inst_so[6]), .C(n476), .Z(n224) );
  HS65_GS_NAND3X5 U439 ( .A(n183), .B(n435), .C(irq[11]), .Z(n178) );
  HS65_GS_NAND3AX6 U440 ( .A(inst_so[5]), .B(n476), .C(n469), .Z(n240) );
  HS65_GS_IVX9 U441 ( .A(n335), .Z(inst_as[1]) );
  HS65_GS_IVX9 U442 ( .A(inst_so[7]), .Z(n476) );
  HS65_GS_IVX9 U443 ( .A(inst_src_bin[1]), .Z(n470) );
  HS65_GS_IVX9 U444 ( .A(inst_dest_bin[1]), .Z(n488) );
  HS65_GS_IVX9 U445 ( .A(inst_dest_bin[0]), .Z(n489) );
  HS65_GS_NAND3X5 U446 ( .A(n174), .B(n178), .C(irq[9]), .Z(n179) );
  HS65_GS_IVX9 U447 ( .A(inst_src_bin[2]), .Z(n484) );
  HS65_GS_NAND4ABX3 U448 ( .A(n271), .B(n272), .C(n273), .D(n274), .Z(
        e_state_nxt[1]) );
  HS65_GS_AOI32X5 U449 ( .A(e_state[3]), .B(e_state[1]), .C(e_state[2]), .D(
        n268), .E(n282), .Z(n273) );
  HS65_GS_NOR4ABX2 U450 ( .A(n142), .B(n283), .C(inst_ad[1]), .D(inst_ad[4]), 
        .Z(n272) );
  HS65_GS_AOI212X4 U451 ( .A(n275), .B(n276), .C(n277), .D(n468), .E(n250), 
        .Z(n274) );
  HS65_GS_NAND3AX6 U452 ( .A(n173), .B(n435), .C(n182), .Z(n177) );
  HS65_GS_OA12X9 U453 ( .A(irq[10]), .B(wdt_irq), .C(n178), .Z(n182) );
  HS65_GS_NOR3X4 U454 ( .A(n225), .B(inst_jmp_bin[1]), .C(inst_jmp_bin[0]), 
        .Z(inst_jmp[4]) );
  HS65_GS_AO312X9 U455 ( .A(n108), .B(n60), .C(n111), .D(exec_jmp), .E(n197), 
        .F(n41), .Z(n428) );
  HS65_GS_IVX9 U456 ( .A(n336), .Z(inst_as[4]) );
  HS65_GS_AO212X4 U457 ( .A(inst_sext[0]), .B(n69), .C(mdb_in[0]), .D(n70), 
        .E(n101), .Z(n370) );
  HS65_GS_OAI21X3 U458 ( .A(n102), .B(n278), .C(n73), .Z(n101) );
  HS65_GS_AO212X4 U459 ( .A(n70), .B(ext_nxt_7_), .C(mdb_in[6]), .D(n81), .E(
        n82), .Z(n363) );
  HS65_GS_AO12X9 U460 ( .A(inst_sext[7]), .B(n69), .C(n286), .Z(n82) );
  HS65_GS_AO212X4 U461 ( .A(n304), .B(OBSERVE_decode), .C(inst_alu[4]), .D(n32), .E(n33), .Z(n344) );
  HS65_GS_IVX9 U462 ( .A(n45), .Z(n304) );
  HS65_GS_MX41X7 U463 ( .D0(n81), .S0(mdb_in[0]), .D1(n99), .S1(n100), .D2(
        ext_nxt_1_), .S2(n70), .D3(inst_sext[1]), .S3(n69), .Z(n369) );
  HS65_GS_AO212X4 U464 ( .A(n31), .B(OBSERVE_decode), .C(inst_alu[0]), .D(n194), .E(n33), .Z(n340) );
  HS65_GS_OAI21X3 U465 ( .A(n439), .B(n296), .C(n36), .Z(n31) );
  HS65_GS_AO212X4 U466 ( .A(n40), .B(OBSERVE_decode), .C(inst_alu[3]), .D(n32), 
        .E(n41), .Z(n343) );
  HS65_GS_AO212X4 U467 ( .A(n38), .B(OBSERVE_decode), .C(inst_alu[2]), .D(n193), .E(n39), .Z(n342) );
  HS65_GS_AO212X4 U468 ( .A(inst_sext[10]), .B(n69), .C(ext_nxt_10_), .D(n70), 
        .E(n71), .Z(n360) );
  HS65_GS_AO212X4 U469 ( .A(inst_sext[11]), .B(n69), .C(ext_nxt_11_), .D(n70), 
        .E(n71), .Z(n359) );
  HS65_GS_AO212X4 U470 ( .A(inst_sext[12]), .B(n69), .C(ext_nxt_12_), .D(n70), 
        .E(n71), .Z(n358) );
  HS65_GS_AO212X4 U471 ( .A(inst_sext[13]), .B(n69), .C(ext_nxt_13_), .D(n70), 
        .E(n71), .Z(n357) );
  HS65_GS_AO212X4 U472 ( .A(inst_sext[14]), .B(n69), .C(ext_nxt_14_), .D(n70), 
        .E(n71), .Z(n356) );
  HS65_GS_AO212X4 U473 ( .A(inst_sext[15]), .B(n69), .C(ext_nxt_15_), .D(n70), 
        .E(n71), .Z(n355) );
  HS65_GS_IVX9 U474 ( .A(n334), .Z(inst_as[2]) );
  HS65_GS_AO32X4 U475 ( .A(n305), .B(mdb_in[13]), .C(n43), .D(inst_alu[6]), 
        .E(n32), .Z(n346) );
  HS65_GS_AO32X4 U476 ( .A(n133), .B(mdb_in[9]), .C(n62), .D(n193), .E(
        inst_so[5]), .Z(n405) );
  HS65_GS_AO112X9 U477 ( .A(inst_as[0]), .B(n32), .C(n81), .D(n126), .Z(n401)
         );
  HS65_GS_NOR4ABX2 U478 ( .A(n127), .B(n445), .C(n193), .D(mdb_in[4]), .Z(n126) );
  HS65_GS_AO22X9 U479 ( .A(inst_alu[10]), .B(n32), .C(n59), .D(OBSERVE_decode), 
        .Z(n350) );
  HS65_GS_AO32X4 U480 ( .A(n114), .B(mdb_in[6]), .C(n115), .D(inst_bw), .E(n32), .Z(n392) );
  HS65_GS_NOR3X4 U481 ( .A(n339), .B(n193), .C(OBSERVE_irq_detect), .Z(n115)
         );
  HS65_GS_AO32X4 U482 ( .A(n305), .B(n438), .C(n48), .D(inst_alu[5]), .E(n32), 
        .Z(n345) );
  HS65_GS_AO32X4 U483 ( .A(n103), .B(OBSERVE_decode), .C(n124), .D(inst_as[3]), 
        .E(n32), .Z(n398) );
  HS65_GS_AO32X4 U484 ( .A(n62), .B(n442), .C(n133), .D(inst_so[1]), .E(n32), 
        .Z(n409) );
  HS65_GS_AO32X4 U485 ( .A(n111), .B(n442), .C(n62), .D(inst_so[0]), .E(n32), 
        .Z(n410) );
  HS65_GS_AO32X4 U486 ( .A(n43), .B(n438), .C(n294), .D(inst_mov), .E(n32), 
        .Z(n390) );
  HS65_GS_IVX9 U487 ( .A(n54), .Z(n294) );
  HS65_GS_AO32X4 U488 ( .A(n103), .B(OBSERVE_decode), .C(n123), .D(n32), .E(
        inst_as[5]), .Z(n396) );
  HS65_GS_AO22X9 U489 ( .A(inst_dext[1]), .B(n106), .C(ext_nxt_1_), .D(n458), 
        .Z(n385) );
  HS65_GS_AO22X9 U490 ( .A(inst_ad[0]), .B(n32), .C(n111), .D(n60), .Z(n389)
         );
  HS65_GS_AO22X9 U491 ( .A(n194), .B(inst_as[6]), .C(n118), .D(n122), .Z(n395)
         );
  HS65_GS_AO22X9 U492 ( .A(n194), .B(inst_sz[1]), .C(n66), .D(n67), .Z(n353)
         );
  HS65_GS_AO22X9 U493 ( .A(inst_dext[2]), .B(n106), .C(ext_nxt_2_), .D(n458), 
        .Z(n384) );
  HS65_GS_AO22X9 U494 ( .A(inst_as[4]), .B(n32), .C(n123), .D(n122), .Z(n397)
         );
  HS65_GS_AO22X9 U495 ( .A(inst_dext[3]), .B(n106), .C(ext_nxt_3_), .D(n458), 
        .Z(n383) );
  HS65_GS_AO22X9 U496 ( .A(inst_dext[8]), .B(n106), .C(ext_nxt_8_), .D(n458), 
        .Z(n378) );
  HS65_GS_AO22X9 U497 ( .A(inst_dext[6]), .B(n106), .C(ext_nxt_6_), .D(n458), 
        .Z(n380) );
  HS65_GS_AO22X9 U498 ( .A(inst_dext[5]), .B(n106), .C(ext_nxt_5_), .D(n458), 
        .Z(n381) );
  HS65_GS_AO22X9 U499 ( .A(inst_dext[9]), .B(n106), .C(ext_nxt_9_), .D(n458), 
        .Z(n377) );
  HS65_GS_AO22X9 U500 ( .A(inst_dext[4]), .B(n106), .C(ext_nxt_4_), .D(n458), 
        .Z(n382) );
  HS65_GS_AO22X9 U501 ( .A(inst_alu[8]), .B(n32), .C(n50), .D(OBSERVE_decode), 
        .Z(n348) );
  HS65_GS_AO22X9 U502 ( .A(inst_as[1]), .B(n32), .C(n122), .D(n124), .Z(n400)
         );
  HS65_GS_AO22X9 U503 ( .A(n194), .B(inst_ad[4]), .C(n67), .D(n108), .Z(n387)
         );
  HS65_GS_AO22X9 U504 ( .A(n194), .B(inst_ad[6]), .C(n67), .D(n68), .Z(n354)
         );
  HS65_GS_AO22X9 U505 ( .A(inst_dext[7]), .B(n106), .C(ext_nxt_7_), .D(n458), 
        .Z(n379) );
  HS65_GS_AO22X9 U506 ( .A(inst_dext[14]), .B(n106), .C(ext_nxt_14_), .D(n458), 
        .Z(n372) );
  HS65_GS_AO22X9 U507 ( .A(inst_dext[10]), .B(n106), .C(ext_nxt_10_), .D(n458), 
        .Z(n376) );
  HS65_GS_AO22X9 U508 ( .A(inst_dext[13]), .B(n106), .C(ext_nxt_13_), .D(n458), 
        .Z(n373) );
  HS65_GS_AO22X9 U509 ( .A(inst_dext[15]), .B(n106), .C(ext_nxt_15_), .D(n458), 
        .Z(n371) );
  HS65_GS_AO22X9 U510 ( .A(inst_dext[11]), .B(n106), .C(ext_nxt_11_), .D(n458), 
        .Z(n375) );
  HS65_GS_AO22X9 U511 ( .A(inst_dext[12]), .B(n106), .C(ext_nxt_12_), .D(n458), 
        .Z(n374) );
  HS65_GS_AO22X9 U516 ( .A(n194), .B(inst_type[0]), .C(OBSERVE_decode), .D(
        n318), .Z(n412) );
  HS65_GS_AO22X9 U517 ( .A(inst_alu[11]), .B(n32), .C(n48), .D(n298), .Z(n351)
         );
  HS65_GS_AO12X9 U518 ( .A(inst_alu[7]), .B(n32), .C(n39), .Z(n347) );
  HS65_GS_AO22X9 U519 ( .A(n194), .B(inst_type[2]), .C(OBSERVE_decode), .D(n60), .Z(n391) );
  HS65_GS_AO22X9 U520 ( .A(n194), .B(inst_sz[0]), .C(OBSERVE_decode), .D(n65), 
        .Z(n352) );
  HS65_GS_AO12X9 U521 ( .A(inst_as[7]), .B(n194), .C(n99), .Z(n393) );
  HS65_GS_AO12X9 U522 ( .A(n32), .B(inst_jmp_bin[2]), .C(n48), .Z(n425) );
  HS65_GS_NOR2X6 U523 ( .A(n482), .B(e_state[1]), .Z(n249) );
  HS65_GS_IVX9 U524 ( .A(e_state[2]), .Z(n480) );
  HS65_GS_OAI22X6 U525 ( .A(n197), .B(n471), .C(exec_jmp), .D(n328), .Z(
        exec_done) );
  HS65_GS_AOI22X6 U526 ( .A(n329), .B(n481), .C(exec_dst_wr), .D(n453), .Z(
        n328) );
  HS65_GS_NOR3X4 U527 ( .A(n468), .B(e_state[2]), .C(n482), .Z(n330) );
  HS65_GS_NAND3X5 U528 ( .A(e_state[2]), .B(n454), .C(n249), .Z(n197) );
  HS65_GS_NAND2X7 U529 ( .A(e_state[0]), .B(n330), .Z(n257) );
  HS65_GS_IVX9 U530 ( .A(e_state[1]), .Z(n468) );
  HS65_GS_AO222X4 U531 ( .A(mdb_in[0]), .B(n12), .C(pc[0]), .D(n9), .E(
        pc_sw[0]), .F(pc_sw_wr), .Z(pc_nxt[0]) );
  HS65_GS_IVX9 U532 ( .A(e_state[3]), .Z(n482) );
  HS65_GS_NOR2X6 U533 ( .A(n480), .B(e_state[3]), .Z(n143) );
  HS65_GS_IVX9 U534 ( .A(e_state[0]), .Z(n454) );
  HS65_GS_AO22X9 U535 ( .A(inst_dext[0]), .B(n106), .C(mdb_in[0]), .D(n458), 
        .Z(n386) );
  HS65_GS_NOR3X4 U536 ( .A(n460), .B(OBSERVE_i_state[2]), .C(n474), .Z(n290)
         );
  HS65_GS_NOR3X4 U537 ( .A(OBSERVE_i_state[0]), .B(OBSERVE_i_state[1]), .C(
        n473), .Z(n291) );
  HS65_GS_NOR3X4 U538 ( .A(e_state[2]), .B(e_state[3]), .C(e_state[0]), .Z(
        n277) );
  HS65_GS_NAND3X5 U539 ( .A(e_state[2]), .B(e_state[0]), .C(n249), .Z(n299) );
  HS65_GS_NOR3X4 U540 ( .A(OBSERVE_i_state[0]), .B(OBSERVE_i_state[2]), .C(
        n474), .Z(n246) );
  HS65_GS_NOR3X4 U541 ( .A(n460), .B(OBSERVE_i_state[1]), .C(n473), .Z(n308)
         );
  HS65_GS_NOR2X6 U542 ( .A(n479), .B(e_state[1]), .Z(n268) );
  HS65_GS_NOR2X6 U543 ( .A(n479), .B(e_state[0]), .Z(n142) );
  HS65_GS_NOR3X4 U544 ( .A(n257), .B(exec_jmp), .C(n478), .Z(n271) );
  HS65_GS_NAND3X5 U545 ( .A(e_state[0]), .B(n480), .C(n249), .Z(n199) );
  HS65_GS_OAI21X3 U546 ( .A(exec_jmp), .B(n256), .C(n270), .Z(n276) );
  HS65_GS_IVX9 U547 ( .A(exec_dst_wr), .Z(n481) );
  HS65_GS_IVX9 U548 ( .A(OBSERVE_i_state[0]), .Z(n460) );
  HS65_GS_IVX9 U549 ( .A(OBSERVE_i_state[2]), .Z(n473) );
  HS65_GS_IVX9 U550 ( .A(OBSERVE_i_state[1]), .Z(n474) );
  HS65_GS_NOR2X6 U551 ( .A(exec_src_wr), .B(exec_jmp), .Z(n263) );
  HS65_GS_NOR2X6 U552 ( .A(exec_dext_rdy), .B(n458), .Z(n121) );
  HS65_GS_IVX9 U553 ( .A(exec_src_wr), .Z(n478) );
  HS65_GS_IVX9 U554 ( .A(exec_jmp), .Z(n471) );
  HS65_GS_NOR2X6 U555 ( .A(inst_ad[6]), .B(inst_ad[4]), .Z(n292) );
  HS65_GS_OA311X9 U556 ( .A(n472), .B(exec_dst_wr), .C(n257), .D(n299), .E(
        n300), .Z(n270) );
  HS65_GS_IVX9 U557 ( .A(n263), .Z(n472) );
  HS65_GS_NOR2AX3 U558 ( .A(n197), .B(n145), .Z(n300) );
  HS65_GS_IVX9 U559 ( .A(n333), .Z(inst_so[3]) );
  HS65_GS_NOR3X4 U560 ( .A(OBSERVE_i_state[1]), .B(OBSERVE_i_state[2]), .C(
        n460), .Z(n25) );
  HS65_GS_NOR2AX3 U561 ( .A(inst_sz[0]), .B(inst_sz[1]), .Z(n310) );
  HS65_GS_AOI311X4 U562 ( .A(inst_type[0]), .B(e_state[1]), .C(n143), .D(n453), 
        .E(n145), .Z(n140) );
  HS65_GS_NOR4ABX2 U563 ( .A(n469), .B(n477), .C(inst_ad[6]), .D(inst_so[5]), 
        .Z(n283) );
  HS65_GS_OAI21X3 U564 ( .A(n104), .B(n456), .C(e_state[0]), .Z(n282) );
  HS65_GS_AND2X4 U565 ( .A(inst_irq_rst), .B(n452), .Z(n338) );
  HS65_GS_NOR2AX3 U566 ( .A(cpu_en_s), .B(cpu_halt_cmd), .Z(n114) );
  HS65_GS_NAND3X5 U567 ( .A(dbg_reg_sel[0]), .B(cpu_halt_st), .C(
        dbg_reg_sel[3]), .Z(n228) );
  HS65_GS_NAND3X5 U568 ( .A(cpu_halt_st), .B(n494), .C(dbg_reg_sel[3]), .Z(
        n230) );
  HS65_GS_NAND3X5 U569 ( .A(cpu_halt_st), .B(n497), .C(dbg_reg_sel[0]), .Z(
        n232) );
  HS65_GS_IVX9 U570 ( .A(nmi_pnd), .Z(n435) );
  HS65_GS_NAND2X7 U571 ( .A(dbg_reg_sel[2]), .B(dbg_reg_sel[1]), .Z(n233) );
  HS65_GS_NAND2X7 U572 ( .A(dbg_reg_sel[2]), .B(n495), .Z(n235) );
  HS65_GS_NAND2X7 U573 ( .A(dbg_reg_sel[1]), .B(n496), .Z(n238) );
  HS65_GS_IVX9 U574 ( .A(cpuoff), .Z(n430) );
  HS65_GS_IVX9 U575 ( .A(dbg_reg_sel[0]), .Z(n494) );
  HS65_GS_IVX9 U576 ( .A(dbg_reg_sel[3]), .Z(n497) );
  HS65_GS_IVX9 U577 ( .A(dbg_reg_sel[1]), .Z(n495) );
  HS65_GS_IVX9 U578 ( .A(dbg_reg_sel[2]), .Z(n496) );
  HS65_GS_BFX9 U579 ( .A(mab[13]), .Z(pc_nxt[13]) );
  HS65_GS_AO112X9 U580 ( .A(pc_incr_13_), .B(n9), .C(n10), .D(n19), .Z(mab[13]) );
  HS65_GS_BFX9 U581 ( .A(mab[12]), .Z(pc_nxt[12]) );
  HS65_GS_AO112X9 U582 ( .A(pc_incr_12_), .B(n9), .C(n10), .D(n20), .Z(mab[12]) );
  HS65_GS_BFX9 U583 ( .A(mab[11]), .Z(pc_nxt[11]) );
  HS65_GS_AO112X9 U584 ( .A(pc_incr_11_), .B(n9), .C(n10), .D(n21), .Z(mab[11]) );
  HS65_GS_BFX9 U585 ( .A(mab[10]), .Z(pc_nxt[10]) );
  HS65_GS_AO112X9 U586 ( .A(pc_incr_10_), .B(n9), .C(n10), .D(n22), .Z(mab[10]) );
  HS65_GS_BFX9 U587 ( .A(mab[9]), .Z(pc_nxt[9]) );
  HS65_GS_AO112X9 U588 ( .A(pc_incr_9_), .B(n9), .C(n10), .D(n11), .Z(mab[9])
         );
  HS65_GS_BFX9 U589 ( .A(mab[8]), .Z(pc_nxt[8]) );
  HS65_GS_AO112X9 U590 ( .A(pc_incr_8_), .B(n9), .C(n10), .D(n13), .Z(mab[8])
         );
  HS65_GS_BFX9 U591 ( .A(mab[7]), .Z(pc_nxt[7]) );
  HS65_GS_AO112X9 U592 ( .A(pc_incr_7_), .B(n9), .C(n10), .D(n14), .Z(mab[7])
         );
  HS65_GS_BFX9 U593 ( .A(mab[6]), .Z(pc_nxt[6]) );
  HS65_GS_AO112X9 U594 ( .A(pc_incr_6_), .B(n9), .C(n10), .D(n15), .Z(mab[6])
         );
  HS65_GS_BFX9 U595 ( .A(mab[5]), .Z(pc_nxt[5]) );
  HS65_GS_AO112X9 U596 ( .A(pc_incr_5_), .B(n9), .C(n10), .D(n16), .Z(mab[5])
         );
  HS65_GS_BFX9 U597 ( .A(mab[4]), .Z(pc_nxt[4]) );
  HS65_GS_MX41X7 U598 ( .D0(pc_sw[4]), .S0(pc_sw_wr), .D1(mdb_in[4]), .S1(n12), 
        .D2(pc_incr_4_), .S2(n9), .D3(OBSERVE_irq_num[3]), .S3(n10), .Z(mab[4]) );
  HS65_GS_BFX9 U599 ( .A(mab[3]), .Z(pc_nxt[3]) );
  HS65_GS_MX41X7 U600 ( .D0(pc_sw[3]), .S0(pc_sw_wr), .D1(mdb_in[3]), .S1(n12), 
        .D2(pc_incr_3_), .S2(n9), .D3(OBSERVE_irq_num[2]), .S3(n10), .Z(mab[3]) );
  HS65_GS_BFX9 U601 ( .A(mab[2]), .Z(pc_nxt[2]) );
  HS65_GS_MX41X7 U602 ( .D0(pc_sw[2]), .S0(pc_sw_wr), .D1(mdb_in[2]), .S1(n12), 
        .D2(pc_incr_2_), .S2(n9), .D3(OBSERVE_irq_num[1]), .S3(n10), .Z(mab[2]) );
  HS65_GS_BFX9 U603 ( .A(mab[1]), .Z(pc_nxt[1]) );
  HS65_GS_MX41X7 U604 ( .D0(pc_sw[1]), .S0(pc_sw_wr), .D1(mdb_in[1]), .S1(n12), 
        .D2(pc_incr_1_), .S2(n9), .D3(OBSERVE_irq_num[0]), .S3(n10), .Z(mab[1]) );
  HS65_GS_AND2X4 U605 ( .A(mdb_in[1]), .B(N702), .Z(add_477_carry[2]) );
  HS65_GSS_XOR2X3 U606 ( .A(mdb_in[1]), .B(N702), .Z(ext_nxt_1_) );
  HS65_GSS_XOR2X3 U607 ( .A(pc[15]), .B(add_428_carry[15]), .Z(pc_incr_15_) );
  HS65_GS_AND2X4 U608 ( .A(add_428_carry[14]), .B(pc[14]), .Z(
        add_428_carry[15]) );
  HS65_GSS_XOR2X3 U609 ( .A(add_428_carry[14]), .B(pc[14]), .Z(pc_incr_14_) );
  HS65_GS_AND2X4 U610 ( .A(add_428_carry[13]), .B(pc[13]), .Z(
        add_428_carry[14]) );
  HS65_GSS_XOR2X3 U611 ( .A(add_428_carry[13]), .B(pc[13]), .Z(pc_incr_13_) );
  HS65_GS_AND2X4 U612 ( .A(add_428_carry[12]), .B(pc[12]), .Z(
        add_428_carry[13]) );
  HS65_GSS_XOR2X3 U613 ( .A(add_428_carry[12]), .B(pc[12]), .Z(pc_incr_12_) );
  HS65_GS_AND2X4 U614 ( .A(add_428_carry[11]), .B(pc[11]), .Z(
        add_428_carry[12]) );
  HS65_GSS_XOR2X3 U615 ( .A(add_428_carry[11]), .B(pc[11]), .Z(pc_incr_11_) );
  HS65_GS_AND2X4 U616 ( .A(add_428_carry[10]), .B(pc[10]), .Z(
        add_428_carry[11]) );
  HS65_GSS_XOR2X3 U617 ( .A(add_428_carry[10]), .B(pc[10]), .Z(pc_incr_10_) );
  HS65_GS_AND2X4 U618 ( .A(add_428_carry[9]), .B(pc[9]), .Z(add_428_carry[10])
         );
  HS65_GSS_XOR2X3 U619 ( .A(add_428_carry[9]), .B(pc[9]), .Z(pc_incr_9_) );
  HS65_GS_AND2X4 U620 ( .A(add_428_carry[8]), .B(pc[8]), .Z(add_428_carry[9])
         );
  HS65_GSS_XOR2X3 U621 ( .A(add_428_carry[8]), .B(pc[8]), .Z(pc_incr_8_) );
  HS65_GS_AND2X4 U622 ( .A(add_428_carry[7]), .B(pc[7]), .Z(add_428_carry[8])
         );
  HS65_GSS_XOR2X3 U623 ( .A(add_428_carry[7]), .B(pc[7]), .Z(pc_incr_7_) );
  HS65_GS_AND2X4 U624 ( .A(add_428_carry[6]), .B(pc[6]), .Z(add_428_carry[7])
         );
  HS65_GSS_XOR2X3 U625 ( .A(add_428_carry[6]), .B(pc[6]), .Z(pc_incr_6_) );
  HS65_GS_AND2X4 U626 ( .A(add_428_carry[5]), .B(pc[5]), .Z(add_428_carry[6])
         );
  HS65_GSS_XOR2X3 U627 ( .A(add_428_carry[5]), .B(pc[5]), .Z(pc_incr_5_) );
  HS65_GS_AND2X4 U628 ( .A(add_428_carry[4]), .B(pc[4]), .Z(add_428_carry[5])
         );
  HS65_GSS_XOR2X3 U629 ( .A(add_428_carry[4]), .B(pc[4]), .Z(pc_incr_4_) );
  HS65_GS_AND2X4 U630 ( .A(add_428_carry[3]), .B(pc[3]), .Z(add_428_carry[4])
         );
  HS65_GSS_XOR2X3 U631 ( .A(add_428_carry[3]), .B(pc[3]), .Z(pc_incr_3_) );
  HS65_GS_AND2X4 U632 ( .A(add_428_carry[2]), .B(pc[2]), .Z(add_428_carry[3])
         );
  HS65_GSS_XOR2X3 U633 ( .A(add_428_carry[2]), .B(pc[2]), .Z(pc_incr_2_) );
  HS65_GS_AND2X4 U634 ( .A(pc[1]), .B(fetch), .Z(add_428_carry[2]) );
  HS65_GSS_XOR2X3 U635 ( .A(pc[1]), .B(fetch), .Z(pc_incr_1_) );
endmodule


module omsp_register_file ( cpuoff, gie, oscoff, pc_sw, pc_sw_wr, reg_dest, 
        reg_src, scg0, scg1, status, OBSERVE_r0, OBSERVE_r1, OBSERVE_r2, 
        OBSERVE_r3, OBSERVE_r4, OBSERVE_r5, OBSERVE_r6, OBSERVE_r7, OBSERVE_r8, 
        OBSERVE_r9, OBSERVE_r10, OBSERVE_r11, OBSERVE_r12, OBSERVE_r13, 
        OBSERVE_r14, OBSERVE_r15, alu_stat, alu_stat_wr, inst_bw, inst_dest, 
        inst_src, mclk, pc, puc_rst, reg_dest_val, reg_dest_wr, reg_pc_call, 
        reg_sp_val, reg_sp_wr, reg_sr_wr, reg_sr_clr, reg_incr, scan_enable );
  output [15:0] pc_sw;
  output [15:0] reg_dest;
  output [15:0] reg_src;
  output [3:0] status;
  output [15:0] OBSERVE_r0;
  output [15:0] OBSERVE_r1;
  output [15:0] OBSERVE_r2;
  output [15:0] OBSERVE_r3;
  output [15:0] OBSERVE_r4;
  output [15:0] OBSERVE_r5;
  output [15:0] OBSERVE_r6;
  output [15:0] OBSERVE_r7;
  output [15:0] OBSERVE_r8;
  output [15:0] OBSERVE_r9;
  output [15:0] OBSERVE_r10;
  output [15:0] OBSERVE_r11;
  output [15:0] OBSERVE_r12;
  output [15:0] OBSERVE_r13;
  output [15:0] OBSERVE_r14;
  output [15:0] OBSERVE_r15;
  input [3:0] alu_stat;
  input [3:0] alu_stat_wr;
  input [15:0] inst_dest;
  input [15:0] inst_src;
  input [15:0] pc;
  input [15:0] reg_dest_val;
  input [15:0] reg_sp_val;
  input inst_bw, mclk, puc_rst, reg_dest_wr, reg_pc_call, reg_sp_wr, reg_sr_wr,
         reg_sr_clr, reg_incr, scan_enable;
  output cpuoff, gie, oscoff, pc_sw_wr, scg0, scg1;
  wire   incr_op_1_, N76, N77, N78, N79, N84, n3, n4, n5, n6, n9, n10, n12,
         n16, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n30, n31,
         n34, n35, n37, n39, n40, n41, n42, n43, n44, n47, n48, n49, n50, n51,
         n54, n55, n56, n57, n58, n61, n62, n63, n64, n65, n68, n69, n70, n71,
         n73, n75, n77, n80, n81, n82, n83, n84, n87, n90, n91, n92, n93, n94,
         n97, n100, n101, n102, n103, n104, n107, n108, n109, n110, n111, n114,
         n115, n116, n117, n118, n121, n122, n123, n124, n125, n128, n129,
         n130, n131, n132, n135, n136, n137, n138, n139, n142, n143, n144,
         n145, n146, n148, n149, n150, n151, n152, n157, n158, n159, n160,
         n161, n163, n164, n165, n166, n167, n168, n171, n172, n173, n174,
         n175, n178, n179, n180, n181, n182, n185, n186, n187, n188, n189,
         n192, n193, n194, n195, n196, n197, n198, n199, n200, n201, n202,
         n204, n205, n206, n207, n208, n209, n211, n212, n213, n214, n215,
         n216, n219, n220, n221, n222, n223, n226, n227, n228, n229, n230,
         n233, n234, n235, n236, n237, n240, n241, n242, n243, n244, n247,
         n248, n249, n250, n251, n254, n255, n256, n257, n258, n259, n267,
         n268, n269, n270, n271, n272, n273, n274, n276, n277, n278, n309,
         n311, n312, n330, n332, n333, n349, n351, n352, n368, n370, n371,
         n387, n389, n390, n406, n408, n409, n424, n425, n426, n427, n428,
         n429, n432, n434, n435, n448, n450, n451, n463, n464, n465, n467,
         n468, n469, n472, n473, n475, n476, n478, n480, n481, n482, n483,
         n484, n485, n486, n487, n488, n489, n490, n491, n492, n493, n494,
         n495, n496, n497, n498, n499, n500, n501, n502, n503, n504, n505,
         n506, n509, n510, n511, n512, n513, n514, n515, n516, n517, n518,
         n519, n520, n521, n522, n523, n524, n525, n526, n527, n528, n529,
         n530, n531, n532, n533, n534, n535, n536, n537, n538, n539, n540,
         n541, n542, n543, n544, n545, n546, n547, n548, n549, n550, n551,
         n552, n553, n554, n555, n556, n557, n558, n559, n560, n561, n562,
         n563, n564, n565, n566, n567, n568, n569, n570, n571, n572, n573,
         n574, n575, n576, n577, n578, n579, n580, n581, n582, n583, n584,
         n585, n586, n587, n588, n589, n590, n591, n592, n593, n594, n595,
         n596, n597, n598, n599, n600, n601, n602, n603, n604, n605, n606,
         n607, n608, n609, n610, n611, n612, n613, n614, n615, n616, n617,
         n618, n619, n620, n621, n622, n623, n624, n625, n626, n627, n628,
         n629, n630, n631, n632, n633, n634, n635, n636, n637, n638, n639,
         n640, n641, n642, n643, n644, n645, n646, n647, n648, n649, n650,
         n651, n652, n653, n654, n655, n656, n657, n658, n659, n660, n661,
         n662, n663, n664, n665, n666, n667, n668, n669, n670, n671, n672,
         n673, n674, n675, n676, n677, n678, n679, n680, n681, n682, n683,
         n684, n685, n686, n687, n688, n689, n690, n691, n692, n693, n694,
         n695, n696, n697, n698, n699, n700, n701, n702, n703, n704, n705,
         n706, n707, n708, n709, n710, n711, n712, n713, n714, n715,
         add_155_B_0_, n96, n98, n99, n105, n106, n112, n113, n119, n120, n126,
         n127, n133, n252, n253, n260, n261, n262, n263, n264, n265, n266,
         n275, n279, n280, n281, n282, n283, n284, n285, n286, n287, n288,
         n289, n290, n291, n292, n293, n294, n295, n296, n297, n298, n299,
         n300, n301, n302, n303, n304, n305, n306, n307, n308, n310, n313,
         n314, n315, n316, n317, n318, n319, n320, n321, n322, n323, n324,
         n325, n326, n327, n328, n329, n331, n334, n335, n336, n337, n338,
         n339, n340, n341, n342, n343, n344, n345, n346, n347, n348, n350,
         n353, n354, n355, n356, n357, n358, n359, n360, n361, n362, n363,
         n364, n365, n366, n367, n369, n372, n373, n374, n375, n376, n377,
         n378, n379, n380, n381, n382, n383, n384, n385, n386, n388, n391,
         n392, n393, n394, n395, n396, n397, n398, n399, n400, n401, n402,
         n403, n404, n405, n407, n410, n411, n412, n413, n414, n415, n416,
         n417, n418, n419, n420, n421, n422, n423, n430, n431, n433, n436,
         n437, n438, n439, n440, n441, n442, n443, n444, n445, n446, n447,
         n449, n452, n453, n454, n455, n456, n457, n458, n459, n460, n461,
         n462, n466, n470, n471, n474, n477, n479, n507, n508, n716, n717,
         n718, n719, n720, n721, n722, n723, n724, n725, n726, n727, n728,
         n729, n730, n731, n732, n733, n734, n735, n736, n737, n738, n739,
         n740, n741, n742, n743, n744, n745, n746, n747, n748, n749, n750,
         n751, n752, n753, n754, n755, n756, n757, n758, n759, n760, n761,
         n762, n763, n764, n765, n766, n767, n768, n769, n770, n771, n772,
         n773, n774, n775, n776, n777, n778, n779, n780, n781, n782, n783,
         n784, n785;
  wire   [15:1] reg_incr_val;
  wire   [15:1] add_155_carry;

  HS65_GS_DFPRQNX4 r8_reg_0_ ( .D(n635), .CP(mclk), .RN(n119), .QN(n490) );
  HS65_GS_DFPRQNX4 r8_reg_1_ ( .D(n634), .CP(mclk), .RN(n119), .QN(n489) );
  HS65_GS_DFPRQNX4 r8_reg_2_ ( .D(n633), .CP(mclk), .RN(n771), .QN(n488) );
  HS65_GS_DFPRQNX4 r8_reg_8_ ( .D(n627), .CP(mclk), .RN(n113), .QN(n487) );
  HS65_GS_DFPRQX4 r3_reg_0_ ( .D(n491), .CP(mclk), .RN(n112), .Q(OBSERVE_r3[0]) );
  HS65_GS_DFPRQX4 r9_reg_0_ ( .D(n619), .CP(mclk), .RN(n119), .Q(OBSERVE_r9[0]) );
  HS65_GS_DFPRQX4 r7_reg_0_ ( .D(n651), .CP(mclk), .RN(n99), .Q(OBSERVE_r7[0])
         );
  HS65_GS_DFPRQX4 r5_reg_0_ ( .D(n683), .CP(mclk), .RN(n96), .Q(OBSERVE_r5[0])
         );
  HS65_GS_DFPRQX4 r6_reg_0_ ( .D(n667), .CP(mclk), .RN(n106), .Q(OBSERVE_r6[0]) );
  HS65_GS_DFPRQX4 r10_reg_0_ ( .D(n603), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r10[0]) );
  HS65_GS_DFPRQX4 r12_reg_0_ ( .D(n571), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r12[0]) );
  HS65_GS_DFPRQX4 r13_reg_0_ ( .D(n555), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r13[0]) );
  HS65_GS_DFPRQX4 r11_reg_0_ ( .D(n587), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r11[0]) );
  HS65_GS_DFPRQX4 r4_reg_0_ ( .D(n699), .CP(mclk), .RN(n96), .Q(OBSERVE_r4[0])
         );
  HS65_GS_DFPRQX4 r15_reg_0_ ( .D(n715), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r15[0]) );
  HS65_GS_DFPRQX4 r14_reg_0_ ( .D(n539), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r14[0]) );
  HS65_GS_DFPRQX4 r1_reg_1_ ( .D(n714), .CP(mclk), .RN(n112), .Q(OBSERVE_r1[1]) );
  HS65_GS_DFPRQX4 r3_reg_1_ ( .D(n492), .CP(mclk), .RN(n120), .Q(OBSERVE_r3[1]) );
  HS65_GS_DFPRQX4 r9_reg_1_ ( .D(n618), .CP(mclk), .RN(n105), .Q(OBSERVE_r9[1]) );
  HS65_GS_DFPRQX4 r7_reg_1_ ( .D(n650), .CP(mclk), .RN(n126), .Q(OBSERVE_r7[1]) );
  HS65_GS_DFPRQX4 r5_reg_1_ ( .D(n682), .CP(mclk), .RN(n106), .Q(OBSERVE_r5[1]) );
  HS65_GS_DFPRQX4 r6_reg_1_ ( .D(n666), .CP(mclk), .RN(n120), .Q(OBSERVE_r6[1]) );
  HS65_GS_DFPRQX4 r10_reg_1_ ( .D(n602), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r10[1]) );
  HS65_GS_DFPRQX4 r12_reg_1_ ( .D(n570), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[1]) );
  HS65_GS_DFPRQX4 r14_reg_1_ ( .D(n538), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r14[1]) );
  HS65_GS_DFPRQX4 r13_reg_1_ ( .D(n554), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r13[1]) );
  HS65_GS_DFPRQX4 r11_reg_1_ ( .D(n586), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r11[1]) );
  HS65_GS_DFPRQX4 r4_reg_1_ ( .D(n698), .CP(mclk), .RN(n98), .Q(OBSERVE_r4[1])
         );
  HS65_GS_DFPRQX4 r15_reg_1_ ( .D(n523), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r15[1]) );
  HS65_GS_DFPRQX4 r1_reg_2_ ( .D(n713), .CP(mclk), .RN(n127), .Q(OBSERVE_r1[2]) );
  HS65_GS_DFPRQX4 r2_reg_3_ ( .D(N79), .CP(mclk), .RN(n99), .Q(gie) );
  HS65_GS_DFPRQX4 r3_reg_2_ ( .D(n493), .CP(mclk), .RN(n105), .Q(OBSERVE_r3[2]) );
  HS65_GS_DFPRQX4 r3_reg_3_ ( .D(n494), .CP(mclk), .RN(n106), .Q(OBSERVE_r3[3]) );
  HS65_GS_DFPRQX4 r1_reg_3_ ( .D(n712), .CP(mclk), .RN(n105), .Q(OBSERVE_r1[3]) );
  HS65_GS_DFPRQX4 r9_reg_2_ ( .D(n617), .CP(mclk), .RN(n771), .Q(OBSERVE_r9[2]) );
  HS65_GS_DFPRQX4 r7_reg_2_ ( .D(n649), .CP(mclk), .RN(n126), .Q(OBSERVE_r7[2]) );
  HS65_GS_DFPRQX4 r5_reg_2_ ( .D(n681), .CP(mclk), .RN(n113), .Q(OBSERVE_r5[2]) );
  HS65_GS_DFPRQX4 r6_reg_2_ ( .D(n665), .CP(mclk), .RN(n127), .Q(OBSERVE_r6[2]) );
  HS65_GS_DFPRQX4 r10_reg_2_ ( .D(n601), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r10[2]) );
  HS65_GS_DFPRQX4 r12_reg_2_ ( .D(n569), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[2]) );
  HS65_GS_DFPRQX4 r14_reg_2_ ( .D(n537), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r14[2]) );
  HS65_GS_DFPRQX4 r13_reg_2_ ( .D(n553), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r13[2]) );
  HS65_GS_DFPRQX4 r11_reg_2_ ( .D(n585), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r11[2]) );
  HS65_GS_DFPRQX4 r4_reg_2_ ( .D(n697), .CP(mclk), .RN(n105), .Q(OBSERVE_r4[2]) );
  HS65_GS_DFPRQX4 r15_reg_2_ ( .D(n522), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r15[2]) );
  HS65_GS_DFPRQX4 r8_reg_3_ ( .D(n632), .CP(mclk), .RN(n126), .Q(OBSERVE_r8[3]) );
  HS65_GS_DFPRQX4 r9_reg_3_ ( .D(n616), .CP(mclk), .RN(n99), .Q(OBSERVE_r9[3])
         );
  HS65_GS_DFPRQX4 r7_reg_3_ ( .D(n648), .CP(mclk), .RN(n126), .Q(OBSERVE_r7[3]) );
  HS65_GS_DFPRQX4 r5_reg_3_ ( .D(n680), .CP(mclk), .RN(n113), .Q(OBSERVE_r5[3]) );
  HS65_GS_DFPRQX4 r6_reg_3_ ( .D(n664), .CP(mclk), .RN(n105), .Q(OBSERVE_r6[3]) );
  HS65_GS_DFPRQX4 r10_reg_3_ ( .D(n600), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r10[3]) );
  HS65_GS_DFPRQX4 r12_reg_3_ ( .D(n568), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r12[3]) );
  HS65_GS_DFPRQX4 r14_reg_3_ ( .D(n536), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r14[3]) );
  HS65_GS_DFPRQX4 r13_reg_3_ ( .D(n552), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r13[3]) );
  HS65_GS_DFPRQX4 r11_reg_3_ ( .D(n584), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r11[3]) );
  HS65_GS_DFPRQX4 r4_reg_3_ ( .D(n696), .CP(mclk), .RN(n119), .Q(OBSERVE_r4[3]) );
  HS65_GS_DFPRQX4 r15_reg_3_ ( .D(n521), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r15[3]) );
  HS65_GS_DFPRQX4 r1_reg_4_ ( .D(n711), .CP(mclk), .RN(n120), .Q(OBSERVE_r1[4]) );
  HS65_GS_DFPRQX4 r3_reg_4_ ( .D(n495), .CP(mclk), .RN(n771), .Q(OBSERVE_r3[4]) );
  HS65_GS_DFPRQX4 r8_reg_4_ ( .D(n631), .CP(mclk), .RN(n126), .Q(OBSERVE_r8[4]) );
  HS65_GS_DFPRQX4 r9_reg_4_ ( .D(n615), .CP(mclk), .RN(n113), .Q(OBSERVE_r9[4]) );
  HS65_GS_DFPRQX4 r7_reg_4_ ( .D(n647), .CP(mclk), .RN(n127), .Q(OBSERVE_r7[4]) );
  HS65_GS_DFPRQX4 r5_reg_4_ ( .D(n679), .CP(mclk), .RN(n120), .Q(OBSERVE_r5[4]) );
  HS65_GS_DFPRQX4 r6_reg_4_ ( .D(n663), .CP(mclk), .RN(n127), .Q(OBSERVE_r6[4]) );
  HS65_GS_DFPRQX4 r10_reg_4_ ( .D(n599), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r10[4]) );
  HS65_GS_DFPRQX4 r12_reg_4_ ( .D(n567), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[4]) );
  HS65_GS_DFPRQX4 r14_reg_4_ ( .D(n535), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r14[4]) );
  HS65_GS_DFPRQX4 r13_reg_4_ ( .D(n551), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r13[4]) );
  HS65_GS_DFPRQX4 r11_reg_4_ ( .D(n583), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r11[4]) );
  HS65_GS_DFPRQX4 r4_reg_4_ ( .D(n695), .CP(mclk), .RN(n127), .Q(OBSERVE_r4[4]) );
  HS65_GS_DFPRQX4 r15_reg_4_ ( .D(n520), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r15[4]) );
  HS65_GS_DFPRQX4 r1_reg_5_ ( .D(n710), .CP(mclk), .RN(n119), .Q(OBSERVE_r1[5]) );
  HS65_GS_DFPRQX4 r3_reg_5_ ( .D(n496), .CP(mclk), .RN(n113), .Q(OBSERVE_r3[5]) );
  HS65_GS_DFPRQX4 r8_reg_5_ ( .D(n630), .CP(mclk), .RN(n98), .Q(OBSERVE_r8[5])
         );
  HS65_GS_DFPRQX4 r9_reg_5_ ( .D(n614), .CP(mclk), .RN(n98), .Q(OBSERVE_r9[5])
         );
  HS65_GS_DFPRQX4 r7_reg_5_ ( .D(n646), .CP(mclk), .RN(n98), .Q(OBSERVE_r7[5])
         );
  HS65_GS_DFPRQX4 r5_reg_5_ ( .D(n678), .CP(mclk), .RN(n112), .Q(OBSERVE_r5[5]) );
  HS65_GS_DFPRQX4 r6_reg_5_ ( .D(n662), .CP(mclk), .RN(n96), .Q(OBSERVE_r6[5])
         );
  HS65_GS_DFPRQX4 r10_reg_5_ ( .D(n598), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r10[5]) );
  HS65_GS_DFPRQX4 r12_reg_5_ ( .D(n566), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r12[5]) );
  HS65_GS_DFPRQX4 r14_reg_5_ ( .D(n534), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r14[5]) );
  HS65_GS_DFPRQX4 r13_reg_5_ ( .D(n550), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r13[5]) );
  HS65_GS_DFPRQX4 r11_reg_5_ ( .D(n582), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r11[5]) );
  HS65_GS_DFPRQX4 r4_reg_5_ ( .D(n694), .CP(mclk), .RN(n105), .Q(OBSERVE_r4[5]) );
  HS65_GS_DFPRQX4 r15_reg_5_ ( .D(n519), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r15[5]) );
  HS65_GS_DFPRQX4 r1_reg_6_ ( .D(n709), .CP(mclk), .RN(n126), .Q(OBSERVE_r1[6]) );
  HS65_GS_DFPRQX4 r3_reg_6_ ( .D(n497), .CP(mclk), .RN(n119), .Q(OBSERVE_r3[6]) );
  HS65_GS_DFPRQX4 r8_reg_6_ ( .D(n629), .CP(mclk), .RN(n127), .Q(OBSERVE_r8[6]) );
  HS65_GS_DFPRQX4 r9_reg_6_ ( .D(n613), .CP(mclk), .RN(n112), .Q(OBSERVE_r9[6]) );
  HS65_GS_DFPRQX4 r7_reg_6_ ( .D(n645), .CP(mclk), .RN(n98), .Q(OBSERVE_r7[6])
         );
  HS65_GS_DFPRQX4 r5_reg_6_ ( .D(n677), .CP(mclk), .RN(n98), .Q(OBSERVE_r5[6])
         );
  HS65_GS_DFPRQX4 r6_reg_6_ ( .D(n661), .CP(mclk), .RN(n98), .Q(OBSERVE_r6[6])
         );
  HS65_GS_DFPRQX4 r10_reg_6_ ( .D(n597), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r10[6]) );
  HS65_GS_DFPRQX4 r12_reg_6_ ( .D(n565), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[6]) );
  HS65_GS_DFPRQX4 r14_reg_6_ ( .D(n533), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r14[6]) );
  HS65_GS_DFPRQX4 r13_reg_6_ ( .D(n549), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r13[6]) );
  HS65_GS_DFPRQX4 r11_reg_6_ ( .D(n581), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r11[6]) );
  HS65_GS_DFPRQX4 r4_reg_6_ ( .D(n693), .CP(mclk), .RN(n99), .Q(OBSERVE_r4[6])
         );
  HS65_GS_DFPRQX4 r15_reg_6_ ( .D(n518), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r15[6]) );
  HS65_GS_DFPRQX4 r1_reg_7_ ( .D(n708), .CP(mclk), .RN(n771), .Q(OBSERVE_r1[7]) );
  HS65_GS_DFPRQX4 r3_reg_7_ ( .D(n498), .CP(mclk), .RN(n112), .Q(OBSERVE_r3[7]) );
  HS65_GS_DFPRQX4 r8_reg_7_ ( .D(n628), .CP(mclk), .RN(n99), .Q(OBSERVE_r8[7])
         );
  HS65_GS_DFPRQX4 r9_reg_7_ ( .D(n612), .CP(mclk), .RN(n99), .Q(OBSERVE_r9[7])
         );
  HS65_GS_DFPRQX4 r7_reg_7_ ( .D(n644), .CP(mclk), .RN(n98), .Q(OBSERVE_r7[7])
         );
  HS65_GS_DFPRQX4 r5_reg_7_ ( .D(n676), .CP(mclk), .RN(n106), .Q(OBSERVE_r5[7]) );
  HS65_GS_DFPRQX4 r6_reg_7_ ( .D(n660), .CP(mclk), .RN(n120), .Q(OBSERVE_r6[7]) );
  HS65_GS_DFPRQX4 r10_reg_7_ ( .D(n596), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r10[7]) );
  HS65_GS_DFPRQX4 r12_reg_7_ ( .D(n564), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r12[7]) );
  HS65_GS_DFPRQX4 r14_reg_7_ ( .D(n532), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r14[7]) );
  HS65_GS_DFPRQX4 r13_reg_7_ ( .D(n548), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r13[7]) );
  HS65_GS_DFPRQX4 r11_reg_7_ ( .D(n580), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r11[7]) );
  HS65_GS_DFPRQX4 r4_reg_7_ ( .D(n692), .CP(mclk), .RN(n126), .Q(OBSERVE_r4[7]) );
  HS65_GS_DFPRQX4 r15_reg_7_ ( .D(n517), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r15[7]) );
  HS65_GS_DFPRQX4 r3_reg_8_ ( .D(n499), .CP(mclk), .RN(n120), .Q(OBSERVE_r3[8]) );
  HS65_GS_DFPRQX4 r9_reg_8_ ( .D(n611), .CP(mclk), .RN(n127), .Q(OBSERVE_r9[8]) );
  HS65_GS_DFPRQX4 r7_reg_8_ ( .D(n643), .CP(mclk), .RN(n127), .Q(OBSERVE_r7[8]) );
  HS65_GS_DFPRQX4 r5_reg_8_ ( .D(n675), .CP(mclk), .RN(n106), .Q(OBSERVE_r5[8]) );
  HS65_GS_DFPRQX4 r6_reg_8_ ( .D(n659), .CP(mclk), .RN(n99), .Q(OBSERVE_r6[8])
         );
  HS65_GS_DFPRQX4 r10_reg_8_ ( .D(n595), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r10[8]) );
  HS65_GS_DFPRQX4 r12_reg_8_ ( .D(n563), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[8]) );
  HS65_GS_DFPRQX4 r14_reg_8_ ( .D(n531), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r14[8]) );
  HS65_GS_DFPRQX4 r13_reg_8_ ( .D(n547), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r13[8]) );
  HS65_GS_DFPRQX4 r11_reg_8_ ( .D(n579), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r11[8]) );
  HS65_GS_DFPRQX4 r4_reg_8_ ( .D(n691), .CP(mclk), .RN(n771), .Q(OBSERVE_r4[8]) );
  HS65_GS_DFPRQX4 r15_reg_8_ ( .D(n516), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r15[8]) );
  HS65_GS_DFPRQX4 r1_reg_8_ ( .D(n707), .CP(mclk), .RN(n105), .Q(OBSERVE_r1[8]) );
  HS65_GS_DFPRQX4 r3_reg_9_ ( .D(n500), .CP(mclk), .RN(n96), .Q(OBSERVE_r3[9])
         );
  HS65_GS_DFPRQX4 r8_reg_9_ ( .D(n626), .CP(mclk), .RN(n127), .Q(OBSERVE_r8[9]) );
  HS65_GS_DFPRQX4 r9_reg_9_ ( .D(n610), .CP(mclk), .RN(n106), .Q(OBSERVE_r9[9]) );
  HS65_GS_DFPRQX4 r7_reg_9_ ( .D(n642), .CP(mclk), .RN(n127), .Q(OBSERVE_r7[9]) );
  HS65_GS_DFPRQX4 r5_reg_9_ ( .D(n674), .CP(mclk), .RN(n771), .Q(OBSERVE_r5[9]) );
  HS65_GS_DFPRQX4 r6_reg_9_ ( .D(n658), .CP(mclk), .RN(n119), .Q(OBSERVE_r6[9]) );
  HS65_GS_DFPRQX4 r10_reg_9_ ( .D(n594), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r10[9]) );
  HS65_GS_DFPRQX4 r12_reg_9_ ( .D(n562), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r12[9]) );
  HS65_GS_DFPRQX4 r14_reg_9_ ( .D(n530), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r14[9]) );
  HS65_GS_DFPRQX4 r13_reg_9_ ( .D(n546), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r13[9]) );
  HS65_GS_DFPRQX4 r11_reg_9_ ( .D(n578), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r11[9]) );
  HS65_GS_DFPRQX4 r4_reg_9_ ( .D(n690), .CP(mclk), .RN(n771), .Q(OBSERVE_r4[9]) );
  HS65_GS_DFPRQX4 r15_reg_9_ ( .D(n515), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r15[9]) );
  HS65_GS_DFPRQX4 r1_reg_9_ ( .D(n706), .CP(mclk), .RN(n126), .Q(OBSERVE_r1[9]) );
  HS65_GS_DFPRQX4 r3_reg_10_ ( .D(n501), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r3[10]) );
  HS65_GS_DFPRQX4 r3_reg_11_ ( .D(n502), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r3[11]) );
  HS65_GS_DFPRQX4 r8_reg_10_ ( .D(n625), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r8[10]) );
  HS65_GS_DFPRQX4 r9_reg_10_ ( .D(n609), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r9[10]) );
  HS65_GS_DFPRQX4 r7_reg_10_ ( .D(n641), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r7[10]) );
  HS65_GS_DFPRQX4 r5_reg_10_ ( .D(n673), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r5[10]) );
  HS65_GS_DFPRQX4 r6_reg_10_ ( .D(n657), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r6[10]) );
  HS65_GS_DFPRQX4 r10_reg_10_ ( .D(n593), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r10[10]) );
  HS65_GS_DFPRQX4 r12_reg_10_ ( .D(n561), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r12[10]) );
  HS65_GS_DFPRQX4 r14_reg_10_ ( .D(n529), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r14[10]) );
  HS65_GS_DFPRQX4 r13_reg_10_ ( .D(n545), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r13[10]) );
  HS65_GS_DFPRQX4 r11_reg_10_ ( .D(n577), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r11[10]) );
  HS65_GS_DFPRQX4 r4_reg_10_ ( .D(n689), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r4[10]) );
  HS65_GS_DFPRQX4 r15_reg_10_ ( .D(n514), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r15[10]) );
  HS65_GS_DFPRQX4 r1_reg_10_ ( .D(n705), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r1[10]) );
  HS65_GS_DFPRQX4 r8_reg_11_ ( .D(n624), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r8[11]) );
  HS65_GS_DFPRQX4 r9_reg_11_ ( .D(n608), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r9[11]) );
  HS65_GS_DFPRQX4 r7_reg_11_ ( .D(n640), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r7[11]) );
  HS65_GS_DFPRQX4 r5_reg_11_ ( .D(n672), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r5[11]) );
  HS65_GS_DFPRQX4 r6_reg_11_ ( .D(n656), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r6[11]) );
  HS65_GS_DFPRQX4 r10_reg_11_ ( .D(n592), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r10[11]) );
  HS65_GS_DFPRQX4 r12_reg_11_ ( .D(n560), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r12[11]) );
  HS65_GS_DFPRQX4 r14_reg_11_ ( .D(n528), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r14[11]) );
  HS65_GS_DFPRQX4 r13_reg_11_ ( .D(n544), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r13[11]) );
  HS65_GS_DFPRQX4 r11_reg_11_ ( .D(n576), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r11[11]) );
  HS65_GS_DFPRQX4 r4_reg_11_ ( .D(n688), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r4[11]) );
  HS65_GS_DFPRQX4 r15_reg_11_ ( .D(n513), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r15[11]) );
  HS65_GS_DFPRQX4 r1_reg_11_ ( .D(n704), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r1[11]) );
  HS65_GS_DFPRQX4 r3_reg_12_ ( .D(n503), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r3[12]) );
  HS65_GS_DFPRQX4 r8_reg_12_ ( .D(n623), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r8[12]) );
  HS65_GS_DFPRQX4 r9_reg_12_ ( .D(n607), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r9[12]) );
  HS65_GS_DFPRQX4 r7_reg_12_ ( .D(n639), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r7[12]) );
  HS65_GS_DFPRQX4 r5_reg_12_ ( .D(n671), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r5[12]) );
  HS65_GS_DFPRQX4 r6_reg_12_ ( .D(n655), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r6[12]) );
  HS65_GS_DFPRQX4 r10_reg_12_ ( .D(n591), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r10[12]) );
  HS65_GS_DFPRQX4 r12_reg_12_ ( .D(n559), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r12[12]) );
  HS65_GS_DFPRQX4 r14_reg_12_ ( .D(n527), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r14[12]) );
  HS65_GS_DFPRQX4 r13_reg_12_ ( .D(n543), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r13[12]) );
  HS65_GS_DFPRQX4 r11_reg_12_ ( .D(n575), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r11[12]) );
  HS65_GS_DFPRQX4 r4_reg_12_ ( .D(n687), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r4[12]) );
  HS65_GS_DFPRQX4 r15_reg_12_ ( .D(n512), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r15[12]) );
  HS65_GS_DFPRQX4 r1_reg_12_ ( .D(n703), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r1[12]) );
  HS65_GS_DFPRQX4 r3_reg_13_ ( .D(n504), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r3[13]) );
  HS65_GS_DFPRQX4 r8_reg_13_ ( .D(n622), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r8[13]) );
  HS65_GS_DFPRQX4 r9_reg_13_ ( .D(n606), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r9[13]) );
  HS65_GS_DFPRQX4 r7_reg_13_ ( .D(n638), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r7[13]) );
  HS65_GS_DFPRQX4 r5_reg_13_ ( .D(n670), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r5[13]) );
  HS65_GS_DFPRQX4 r6_reg_13_ ( .D(n654), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r6[13]) );
  HS65_GS_DFPRQX4 r10_reg_13_ ( .D(n590), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r10[13]) );
  HS65_GS_DFPRQX4 r12_reg_13_ ( .D(n558), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r12[13]) );
  HS65_GS_DFPRQX4 r14_reg_13_ ( .D(n526), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r14[13]) );
  HS65_GS_DFPRQX4 r13_reg_13_ ( .D(n542), .CP(mclk), .RN(n105), .Q(
        OBSERVE_r13[13]) );
  HS65_GS_DFPRQX4 r11_reg_13_ ( .D(n574), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r11[13]) );
  HS65_GS_DFPRQX4 r4_reg_13_ ( .D(n686), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r4[13]) );
  HS65_GS_DFPRQX4 r15_reg_13_ ( .D(n511), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r15[13]) );
  HS65_GS_DFPRQX4 r1_reg_13_ ( .D(n702), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r1[13]) );
  HS65_GS_DFPRQX4 r3_reg_14_ ( .D(n505), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r3[14]) );
  HS65_GS_DFPRQX4 r8_reg_14_ ( .D(n621), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r8[14]) );
  HS65_GS_DFPRQX4 r9_reg_14_ ( .D(n605), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r9[14]) );
  HS65_GS_DFPRQX4 r7_reg_14_ ( .D(n637), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r7[14]) );
  HS65_GS_DFPRQX4 r5_reg_14_ ( .D(n669), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r5[14]) );
  HS65_GS_DFPRQX4 r6_reg_14_ ( .D(n653), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r6[14]) );
  HS65_GS_DFPRQX4 r10_reg_14_ ( .D(n589), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r10[14]) );
  HS65_GS_DFPRQX4 r12_reg_14_ ( .D(n557), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r12[14]) );
  HS65_GS_DFPRQX4 r14_reg_14_ ( .D(n525), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r14[14]) );
  HS65_GS_DFPRQX4 r13_reg_14_ ( .D(n541), .CP(mclk), .RN(n99), .Q(
        OBSERVE_r13[14]) );
  HS65_GS_DFPRQX4 r11_reg_14_ ( .D(n573), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r11[14]) );
  HS65_GS_DFPRQX4 r4_reg_14_ ( .D(n685), .CP(mclk), .RN(n127), .Q(
        OBSERVE_r4[14]) );
  HS65_GS_DFPRQX4 r15_reg_14_ ( .D(n510), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r15[14]) );
  HS65_GS_DFPRQX4 r1_reg_14_ ( .D(n701), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r1[14]) );
  HS65_GS_DFPRQX4 r2_reg_2_ ( .D(N78), .CP(mclk), .RN(n120), .Q(status[2]) );
  HS65_GS_DFPRQX4 r3_reg_15_ ( .D(n506), .CP(mclk), .RN(n119), .Q(
        OBSERVE_r3[15]) );
  HS65_GS_DFPRQX4 r8_reg_15_ ( .D(n620), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r8[15]) );
  HS65_GS_DFPRQX4 r9_reg_15_ ( .D(n604), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r9[15]) );
  HS65_GS_DFPRQX4 r7_reg_15_ ( .D(n636), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r7[15]) );
  HS65_GS_DFPRQX4 r5_reg_15_ ( .D(n668), .CP(mclk), .RN(n120), .Q(
        OBSERVE_r5[15]) );
  HS65_GS_DFPRQX4 r6_reg_15_ ( .D(n652), .CP(mclk), .RN(n113), .Q(
        OBSERVE_r6[15]) );
  HS65_GS_DFPRQX4 r10_reg_15_ ( .D(n588), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r10[15]) );
  HS65_GS_DFPRQX4 r12_reg_15_ ( .D(n556), .CP(mclk), .RN(n96), .Q(
        OBSERVE_r12[15]) );
  HS65_GS_DFPRQX4 r14_reg_15_ ( .D(n524), .CP(mclk), .RN(n771), .Q(
        OBSERVE_r14[15]) );
  HS65_GS_DFPRQX4 r13_reg_15_ ( .D(n540), .CP(mclk), .RN(n112), .Q(
        OBSERVE_r13[15]) );
  HS65_GS_DFPRQX4 r11_reg_15_ ( .D(n572), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r11[15]) );
  HS65_GS_DFPRQX4 r4_reg_15_ ( .D(n684), .CP(mclk), .RN(n98), .Q(
        OBSERVE_r4[15]) );
  HS65_GS_DFPRQX4 r15_reg_15_ ( .D(n509), .CP(mclk), .RN(n126), .Q(
        OBSERVE_r15[15]) );
  HS65_GS_DFPRQX4 r1_reg_15_ ( .D(n700), .CP(mclk), .RN(n106), .Q(
        OBSERVE_r1[15]) );
  HS65_GS_DFPRQX4 r2_reg_8_ ( .D(N84), .CP(mclk), .RN(n126), .Q(status[3]) );
  HS65_GS_DFPRQX4 r2_reg_1_ ( .D(N77), .CP(mclk), .RN(n771), .Q(status[1]) );
  HS65_GS_DFPRQX4 r2_reg_0_ ( .D(N76), .CP(mclk), .RN(n127), .Q(status[0]) );
  HS65_GS_IVX9 U3 ( .A(1'b1), .Z(OBSERVE_r2[4]) );
  HS65_GS_IVX9 U5 ( .A(1'b1), .Z(OBSERVE_r2[5]) );
  HS65_GS_IVX9 U7 ( .A(1'b1), .Z(OBSERVE_r2[6]) );
  HS65_GS_IVX9 U9 ( .A(1'b1), .Z(OBSERVE_r2[7]) );
  HS65_GS_IVX9 U11 ( .A(1'b1), .Z(OBSERVE_r2[9]) );
  HS65_GS_IVX9 U13 ( .A(1'b1), .Z(OBSERVE_r2[10]) );
  HS65_GS_IVX9 U15 ( .A(1'b1), .Z(OBSERVE_r2[11]) );
  HS65_GS_IVX9 U17 ( .A(1'b1), .Z(OBSERVE_r2[12]) );
  HS65_GS_IVX9 U19 ( .A(1'b1), .Z(OBSERVE_r2[13]) );
  HS65_GS_IVX9 U21 ( .A(1'b1), .Z(OBSERVE_r2[14]) );
  HS65_GS_IVX9 U23 ( .A(1'b1), .Z(OBSERVE_r2[15]) );
  HS65_GS_IVX9 U25 ( .A(1'b1), .Z(OBSERVE_r1[0]) );
  HS65_GS_IVX9 U27 ( .A(1'b1), .Z(scg1) );
  HS65_GS_IVX9 U29 ( .A(1'b1), .Z(scg0) );
  HS65_GS_IVX9 U31 ( .A(1'b1), .Z(oscoff) );
  HS65_GS_IVX9 U33 ( .A(1'b1), .Z(cpuoff) );
  HS65_GS_IVX2 U35 ( .A(puc_rst), .Z(n96) );
  HS65_GS_IVX2 U36 ( .A(puc_rst), .Z(n98) );
  HS65_GS_IVX9 U37 ( .A(puc_rst), .Z(n99) );
  HS65_GS_IVX9 U38 ( .A(puc_rst), .Z(n105) );
  HS65_GS_IVX2 U39 ( .A(puc_rst), .Z(n106) );
  HS65_GS_IVX2 U40 ( .A(puc_rst), .Z(n112) );
  HS65_GS_IVX2 U41 ( .A(puc_rst), .Z(n113) );
  HS65_GS_IVX7 U42 ( .A(puc_rst), .Z(n119) );
  HS65_GS_IVX7 U43 ( .A(puc_rst), .Z(n120) );
  HS65_GS_IVX9 U44 ( .A(puc_rst), .Z(n126) );
  HS65_GS_IVX9 U45 ( .A(puc_rst), .Z(n127) );
  HS65_GS_BFX9 U46 ( .A(pc[1]), .Z(OBSERVE_r0[1]) );
  HS65_GS_BFX9 U47 ( .A(pc[2]), .Z(OBSERVE_r0[2]) );
  HS65_GS_BFX9 U48 ( .A(pc[3]), .Z(OBSERVE_r0[3]) );
  HS65_GS_BFX9 U49 ( .A(pc[4]), .Z(OBSERVE_r0[4]) );
  HS65_GS_BFX9 U50 ( .A(pc[5]), .Z(OBSERVE_r0[5]) );
  HS65_GS_BFX9 U51 ( .A(pc[6]), .Z(OBSERVE_r0[6]) );
  HS65_GS_BFX9 U52 ( .A(pc[7]), .Z(OBSERVE_r0[7]) );
  HS65_GS_BFX9 U53 ( .A(pc[8]), .Z(OBSERVE_r0[8]) );
  HS65_GS_BFX9 U54 ( .A(pc[9]), .Z(OBSERVE_r0[9]) );
  HS65_GS_BFX9 U55 ( .A(pc[10]), .Z(OBSERVE_r0[10]) );
  HS65_GS_BFX9 U56 ( .A(pc[11]), .Z(OBSERVE_r0[11]) );
  HS65_GS_BFX9 U57 ( .A(pc[12]), .Z(OBSERVE_r0[12]) );
  HS65_GS_BFX9 U58 ( .A(pc[13]), .Z(OBSERVE_r0[13]) );
  HS65_GS_BFX9 U59 ( .A(pc[14]), .Z(OBSERVE_r0[14]) );
  HS65_GS_BFX9 U60 ( .A(pc[15]), .Z(OBSERVE_r0[15]) );
  HS65_GS_BFX9 U61 ( .A(pc[0]), .Z(OBSERVE_r0[0]) );
  HS65_GS_BFX9 U62 ( .A(status[0]), .Z(OBSERVE_r2[0]) );
  HS65_GS_BFX9 U63 ( .A(status[3]), .Z(OBSERVE_r2[8]) );
  HS65_GS_BFX9 U64 ( .A(status[2]), .Z(OBSERVE_r2[2]) );
  HS65_GS_BFX9 U65 ( .A(status[1]), .Z(OBSERVE_r2[1]) );
  HS65_GS_BFX9 U66 ( .A(gie), .Z(OBSERVE_r2[3]) );
  HS65_GS_IVX9 U67 ( .A(n490), .Z(OBSERVE_r8[0]) );
  HS65_GS_IVX9 U68 ( .A(n489), .Z(OBSERVE_r8[1]) );
  HS65_GS_IVX9 U69 ( .A(n488), .Z(OBSERVE_r8[2]) );
  HS65_GS_IVX9 U70 ( .A(n487), .Z(OBSERVE_r8[8]) );
  HS65_GS_IVX9 U71 ( .A(n274), .Z(pc_sw[15]) );
  HS65_GS_IVX9 U72 ( .A(reg_sr_clr), .Z(n779) );
  HS65_GS_IVX9 U73 ( .A(reg_dest_val[7]), .Z(n285) );
  HS65_GS_IVX9 U74 ( .A(reg_dest_val[5]), .Z(n287) );
  HS65_GS_NAND2X7 U75 ( .A(reg_dest_val[15]), .B(n785), .Z(n274) );
  HS65_GS_NAND2X7 U76 ( .A(reg_dest_val[12]), .B(n785), .Z(n271) );
  HS65_GS_NAND2X7 U77 ( .A(reg_dest_val[13]), .B(n785), .Z(n272) );
  HS65_GS_NAND2X7 U78 ( .A(reg_dest_val[14]), .B(n785), .Z(n273) );
  HS65_GS_NAND2X7 U79 ( .A(reg_dest_val[8]), .B(n785), .Z(n267) );
  HS65_GS_NAND2X7 U80 ( .A(reg_dest_val[9]), .B(n785), .Z(n268) );
  HS65_GS_NAND2X7 U81 ( .A(reg_dest_val[10]), .B(n785), .Z(n269) );
  HS65_GS_NAND2X7 U82 ( .A(reg_dest_val[11]), .B(n785), .Z(n270) );
  HS65_GS_IVX9 U83 ( .A(n12), .Z(n773) );
  HS65_GS_IVX9 U84 ( .A(inst_dest[4]), .Z(n784) );
  HS65_GS_IVX9 U85 ( .A(n35), .Z(n777) );
  HS65_GS_IVX9 U86 ( .A(inst_dest[3]), .Z(n783) );
  HS65_GS_IVX9 U87 ( .A(n37), .Z(n776) );
  HS65_GS_IVX9 U88 ( .A(n31), .Z(n775) );
  HS65_GS_IVX9 U89 ( .A(n472), .Z(n778) );
  HS65_GS_IVX9 U90 ( .A(n10), .Z(n774) );
  HS65_GS_IVX9 U91 ( .A(n75), .Z(n772) );
  HS65_GS_IVX9 U92 ( .A(reg_dest_val[6]), .Z(n286) );
  HS65_GS_IVX9 U93 ( .A(reg_dest_val[3]), .Z(n289) );
  HS65_GS_IVX9 U94 ( .A(reg_dest_val[1]), .Z(n291) );
  HS65_GS_IVX9 U95 ( .A(reg_incr_val[14]), .Z(n260) );
  HS65_GS_IVX9 U96 ( .A(reg_incr_val[13]), .Z(n261) );
  HS65_GS_IVX9 U97 ( .A(reg_incr_val[12]), .Z(n262) );
  HS65_GS_IVX9 U98 ( .A(reg_incr_val[11]), .Z(n263) );
  HS65_GS_IVX9 U99 ( .A(reg_incr_val[10]), .Z(n264) );
  HS65_GS_IVX9 U100 ( .A(reg_incr_val[15]), .Z(n253) );
  HS65_GS_IVX9 U101 ( .A(incr_op_1_), .Z(add_155_B_0_) );
  HS65_GS_NAND2X7 U102 ( .A(n390), .B(n387), .Z(n389) );
  HS65_GS_NAND2X7 U103 ( .A(n371), .B(n368), .Z(n370) );
  HS65_GS_NAND2X7 U104 ( .A(n352), .B(n349), .Z(n351) );
  HS65_GS_NAND2X7 U105 ( .A(n333), .B(n330), .Z(n332) );
  HS65_GS_NAND2X7 U106 ( .A(n312), .B(n309), .Z(n311) );
  HS65_GS_NAND2X7 U107 ( .A(n464), .B(n465), .Z(n463) );
  HS65_GS_NAND2X7 U108 ( .A(n277), .B(n278), .Z(n276) );
  HS65_GS_NAND2X7 U109 ( .A(n451), .B(n448), .Z(n450) );
  HS65_GS_NAND2X7 U110 ( .A(n435), .B(n432), .Z(n434) );
  HS65_GS_NAND2X7 U111 ( .A(n429), .B(n427), .Z(n428) );
  HS65_GS_NAND2X7 U112 ( .A(n426), .B(n424), .Z(n425) );
  HS65_GS_NAND2X7 U113 ( .A(n409), .B(n406), .Z(n408) );
  HS65_GS_IVX9 U114 ( .A(reg_incr_val[9]), .Z(n265) );
  HS65_GS_IVX9 U115 ( .A(reg_incr_val[8]), .Z(n266) );
  HS65_GS_IVX9 U116 ( .A(reg_incr_val[7]), .Z(n275) );
  HS65_GS_IVX9 U117 ( .A(reg_incr_val[6]), .Z(n279) );
  HS65_GS_IVX9 U118 ( .A(reg_incr_val[5]), .Z(n280) );
  HS65_GS_IVX9 U119 ( .A(reg_incr_val[4]), .Z(n281) );
  HS65_GS_IVX9 U120 ( .A(reg_incr_val[3]), .Z(n282) );
  HS65_GS_IVX9 U121 ( .A(reg_incr_val[2]), .Z(n283) );
  HS65_GS_NOR3X4 U122 ( .A(n468), .B(n467), .C(n781), .Z(n469) );
  HS65_GS_IVX9 U123 ( .A(n478), .Z(n782) );
  HS65_GS_IVX9 U124 ( .A(reg_incr_val[1]), .Z(n284) );
  HS65_GS_IVX9 U125 ( .A(n259), .Z(n780) );
  HS65_GS_IVX9 U126 ( .A(n473), .Z(n781) );
  HS65_GS_OAI222X2 U127 ( .A(n770), .B(n463), .C(n274), .D(n464), .E(n253), 
        .F(n465), .Z(n684) );
  HS65_GS_OAI222X2 U128 ( .A(n758), .B(n463), .C(n273), .D(n464), .E(n260), 
        .F(n465), .Z(n685) );
  HS65_GS_OAI222X2 U129 ( .A(n746), .B(n463), .C(n272), .D(n464), .E(n261), 
        .F(n465), .Z(n686) );
  HS65_GS_OAI222X2 U130 ( .A(n253), .B(n427), .C(n767), .D(n428), .E(n274), 
        .F(n429), .Z(n636) );
  HS65_GS_OAI222X2 U131 ( .A(n253), .B(n424), .C(n766), .D(n425), .E(n274), 
        .F(n426), .Z(n620) );
  HS65_GS_OAI222X2 U132 ( .A(n260), .B(n427), .C(n755), .D(n428), .E(n273), 
        .F(n429), .Z(n637) );
  HS65_GS_OAI222X2 U133 ( .A(n260), .B(n424), .C(n754), .D(n425), .E(n273), 
        .F(n426), .Z(n621) );
  HS65_GS_OAI222X2 U134 ( .A(n261), .B(n427), .C(n743), .D(n428), .E(n272), 
        .F(n429), .Z(n638) );
  HS65_GS_OAI222X2 U135 ( .A(n261), .B(n424), .C(n742), .D(n425), .E(n272), 
        .F(n426), .Z(n622) );
  HS65_GS_OAI22X6 U136 ( .A(n780), .B(n296), .C(n259), .D(n272), .Z(n504) );
  HS65_GS_OAI22X6 U137 ( .A(n780), .B(n295), .C(n259), .D(n273), .Z(n505) );
  HS65_GS_OAI22X6 U138 ( .A(n780), .B(n294), .C(n259), .D(n274), .Z(n506) );
  HS65_GS_NOR2AX3 U139 ( .A(inst_src[14]), .B(reg_sr_clr), .Z(n22) );
  HS65_GS_NOR2AX3 U140 ( .A(inst_src[13]), .B(reg_sr_clr), .Z(n21) );
  HS65_GS_NOR2AX3 U141 ( .A(inst_src[10]), .B(reg_sr_clr), .Z(n18) );
  HS65_GS_NOR2AX3 U142 ( .A(inst_src[12]), .B(reg_sr_clr), .Z(n20) );
  HS65_GS_NOR2AX3 U143 ( .A(inst_src[11]), .B(reg_sr_clr), .Z(n19) );
  HS65_GS_NOR2AX3 U144 ( .A(inst_src[3]), .B(reg_sr_clr), .Z(n23) );
  HS65_GS_OAI222X2 U145 ( .A(n734), .B(n463), .C(n271), .D(n464), .E(n262), 
        .F(n465), .Z(n687) );
  HS65_GS_OAI222X2 U146 ( .A(n722), .B(n463), .C(n270), .D(n464), .E(n263), 
        .F(n465), .Z(n688) );
  HS65_GS_OAI222X2 U147 ( .A(n471), .B(n463), .C(n269), .D(n464), .E(n264), 
        .F(n465), .Z(n689) );
  HS65_GS_OAI222X2 U148 ( .A(n453), .B(n463), .C(n268), .D(n464), .E(n265), 
        .F(n465), .Z(n690) );
  HS65_GS_OAI222X2 U149 ( .A(n438), .B(n463), .C(n267), .D(n464), .E(n266), 
        .F(n465), .Z(n691) );
  HS65_GS_OAI222X2 U150 ( .A(n275), .B(n465), .C(n418), .D(n463), .E(n285), 
        .F(n464), .Z(n692) );
  HS65_GS_OAI222X2 U151 ( .A(n279), .B(n465), .C(n403), .D(n463), .E(n286), 
        .F(n464), .Z(n693) );
  HS65_GS_OAI222X2 U152 ( .A(n280), .B(n465), .C(n391), .D(n463), .E(n287), 
        .F(n464), .Z(n694) );
  HS65_GS_OAI222X2 U153 ( .A(n281), .B(n465), .C(n376), .D(n463), .E(n288), 
        .F(n464), .Z(n695) );
  HS65_GS_OAI222X2 U154 ( .A(n282), .B(n465), .C(n361), .D(n463), .E(n289), 
        .F(n464), .Z(n696) );
  HS65_GS_OAI222X2 U155 ( .A(n262), .B(n427), .C(n731), .D(n428), .E(n271), 
        .F(n429), .Z(n639) );
  HS65_GS_OAI222X2 U156 ( .A(n262), .B(n424), .C(n730), .D(n425), .E(n271), 
        .F(n426), .Z(n623) );
  HS65_GS_OAI222X2 U157 ( .A(n263), .B(n427), .C(n719), .D(n428), .E(n270), 
        .F(n429), .Z(n640) );
  HS65_GS_OAI222X2 U158 ( .A(n263), .B(n424), .C(n718), .D(n425), .E(n270), 
        .F(n426), .Z(n624) );
  HS65_GS_OAI222X2 U159 ( .A(n264), .B(n427), .C(n462), .D(n428), .E(n269), 
        .F(n429), .Z(n641) );
  HS65_GS_OAI222X2 U160 ( .A(n264), .B(n424), .C(n461), .D(n425), .E(n269), 
        .F(n426), .Z(n625) );
  HS65_GS_OAI222X2 U161 ( .A(n265), .B(n427), .C(n447), .D(n428), .E(n268), 
        .F(n429), .Z(n642) );
  HS65_GS_OAI222X2 U162 ( .A(n265), .B(n424), .C(n446), .D(n425), .E(n268), 
        .F(n426), .Z(n626) );
  HS65_GS_OAI222X2 U163 ( .A(n266), .B(n448), .C(n437), .D(n450), .E(n267), 
        .F(n451), .Z(n675) );
  HS65_GS_OAI222X2 U164 ( .A(n266), .B(n432), .C(n436), .D(n434), .E(n267), 
        .F(n435), .Z(n659) );
  HS65_GS_OAI222X2 U165 ( .A(n266), .B(n427), .C(n433), .D(n428), .E(n267), 
        .F(n429), .Z(n643) );
  HS65_GS_OAI222X2 U166 ( .A(n275), .B(n427), .C(n415), .D(n428), .E(n285), 
        .F(n429), .Z(n644) );
  HS65_GS_OAI222X2 U167 ( .A(n275), .B(n424), .C(n414), .D(n425), .E(n285), 
        .F(n426), .Z(n628) );
  HS65_GS_OAI222X2 U168 ( .A(n279), .B(n427), .C(n400), .D(n428), .E(n286), 
        .F(n429), .Z(n645) );
  HS65_GS_OAI222X2 U169 ( .A(n279), .B(n424), .C(n399), .D(n425), .E(n286), 
        .F(n426), .Z(n629) );
  HS65_GS_OAI222X2 U170 ( .A(n280), .B(n427), .C(n385), .D(n428), .E(n287), 
        .F(n429), .Z(n646) );
  HS65_GS_OAI222X2 U171 ( .A(n280), .B(n424), .C(n384), .D(n425), .E(n287), 
        .F(n426), .Z(n630) );
  HS65_GS_OAI222X2 U172 ( .A(n281), .B(n427), .C(n373), .D(n428), .E(n288), 
        .F(n429), .Z(n647) );
  HS65_GS_OAI222X2 U173 ( .A(n281), .B(n424), .C(n372), .D(n425), .E(n288), 
        .F(n426), .Z(n631) );
  HS65_GS_OAI222X2 U174 ( .A(n282), .B(n448), .C(n360), .D(n450), .E(n289), 
        .F(n451), .Z(n680) );
  HS65_GS_OAI222X2 U175 ( .A(n282), .B(n424), .C(n357), .D(n425), .E(n289), 
        .F(n426), .Z(n632) );
  HS65_GS_OAI222X2 U176 ( .A(n282), .B(n406), .C(n356), .D(n408), .E(n289), 
        .F(n409), .Z(n616) );
  HS65_GS_NAND2X7 U177 ( .A(inst_src[5]), .B(n779), .Z(n37) );
  HS65_GS_NAND2X7 U178 ( .A(inst_src[4]), .B(n779), .Z(n35) );
  HS65_GS_NAND2X7 U179 ( .A(inst_src[8]), .B(n779), .Z(n12) );
  HS65_GS_NAND2X7 U180 ( .A(inst_src[6]), .B(n779), .Z(n31) );
  HS65_GS_OAI22X6 U181 ( .A(n35), .B(n335), .C(n37), .D(n334), .Z(n97) );
  HS65_GS_OAI22X6 U182 ( .A(n35), .B(n346), .C(n37), .D(n345), .Z(n87) );
  HS65_GS_OAI22X6 U183 ( .A(n35), .B(n438), .C(n37), .D(n437), .Z(n34) );
  HS65_GS_OAI22X6 U184 ( .A(n12), .B(n357), .C(n75), .D(n356), .Z(n73) );
  HS65_GS_OAI22X6 U185 ( .A(n10), .B(n373), .C(n12), .D(n372), .Z(n65) );
  HS65_GS_OAI22X6 U186 ( .A(n10), .B(n385), .C(n12), .D(n384), .Z(n58) );
  HS65_GS_OAI22X6 U187 ( .A(n10), .B(n400), .C(n12), .D(n399), .Z(n51) );
  HS65_GS_OAI22X6 U188 ( .A(n10), .B(n415), .C(n12), .D(n414), .Z(n44) );
  HS65_GS_OAI22X6 U189 ( .A(n10), .B(n447), .C(n12), .D(n446), .Z(n9) );
  HS65_GS_OAI22X6 U190 ( .A(n10), .B(n767), .C(n12), .D(n766), .Z(n104) );
  HS65_GS_OAI22X6 U191 ( .A(n10), .B(n462), .C(n12), .D(n461), .Z(n139) );
  HS65_GS_OAI22X6 U192 ( .A(n10), .B(n719), .C(n12), .D(n718), .Z(n132) );
  HS65_GS_OAI22X6 U193 ( .A(n10), .B(n731), .C(n12), .D(n730), .Z(n125) );
  HS65_GS_OAI22X6 U194 ( .A(n10), .B(n743), .C(n12), .D(n742), .Z(n118) );
  HS65_GS_OAI22X6 U195 ( .A(n10), .B(n755), .C(n12), .D(n754), .Z(n111) );
  HS65_GS_NAND2X7 U196 ( .A(inst_src[7]), .B(n779), .Z(n10) );
  HS65_GS_NAND2X7 U197 ( .A(inst_src[1]), .B(n779), .Z(n472) );
  HS65_GS_IVX9 U198 ( .A(reg_dest_val[4]), .Z(n288) );
  HS65_GS_NAND2X7 U199 ( .A(inst_src[9]), .B(n779), .Z(n75) );
  HS65_GS_OAI22X6 U200 ( .A(n780), .B(n305), .C(n259), .D(n288), .Z(n495) );
  HS65_GS_OAI22X6 U201 ( .A(n780), .B(n304), .C(n259), .D(n287), .Z(n496) );
  HS65_GS_OAI22X6 U202 ( .A(n780), .B(n303), .C(n259), .D(n286), .Z(n497) );
  HS65_GS_OAI22X6 U203 ( .A(n780), .B(n302), .C(n259), .D(n285), .Z(n498) );
  HS65_GS_OAI22X6 U204 ( .A(n780), .B(n301), .C(n259), .D(n267), .Z(n499) );
  HS65_GS_OAI22X6 U205 ( .A(n780), .B(n300), .C(n259), .D(n268), .Z(n500) );
  HS65_GS_OAI22X6 U206 ( .A(n780), .B(n299), .C(n259), .D(n269), .Z(n501) );
  HS65_GS_OAI22X6 U207 ( .A(n780), .B(n298), .C(n259), .D(n270), .Z(n502) );
  HS65_GS_OAI22X6 U208 ( .A(n780), .B(n297), .C(n259), .D(n271), .Z(n503) );
  HS65_GS_BFX9 U209 ( .A(n16), .Z(n252) );
  HS65_GS_NOR2AX3 U210 ( .A(inst_src[15]), .B(reg_sr_clr), .Z(n16) );
  HS65_GS_NAND2AX7 U211 ( .A(inst_src[2]), .B(n779), .Z(n39) );
  HS65_GS_OAI222X2 U212 ( .A(n283), .B(n465), .C(n346), .D(n463), .E(n290), 
        .F(n464), .Z(n697) );
  HS65_GS_OAI222X2 U213 ( .A(n284), .B(n465), .C(n335), .D(n463), .E(n291), 
        .F(n464), .Z(n698) );
  HS65_GS_OAI222X2 U214 ( .A(n283), .B(n448), .C(n345), .D(n450), .E(n290), 
        .F(n451), .Z(n681) );
  HS65_GS_OAI222X2 U215 ( .A(n283), .B(n432), .C(n344), .D(n434), .E(n290), 
        .F(n435), .Z(n665) );
  HS65_GS_OAI222X2 U216 ( .A(n283), .B(n427), .C(n343), .D(n428), .E(n290), 
        .F(n429), .Z(n649) );
  HS65_GS_OAI222X2 U217 ( .A(n284), .B(n448), .C(n334), .D(n450), .E(n291), 
        .F(n451), .Z(n682) );
  HS65_GS_OAI222X2 U218 ( .A(n284), .B(n432), .C(n331), .D(n434), .E(n291), 
        .F(n435), .Z(n666) );
  HS65_GS_OAI222X2 U219 ( .A(n284), .B(n427), .C(n329), .D(n428), .E(n291), 
        .F(n429), .Z(n650) );
  HS65_GS_IVX9 U220 ( .A(reg_dest_val[2]), .Z(n290) );
  HS65_GS_IVX9 U221 ( .A(reg_dest_val[0]), .Z(n292) );
  HS65_GS_OAI22X6 U222 ( .A(n780), .B(n307), .C(n259), .D(n291), .Z(n492) );
  HS65_GS_OAI22X6 U223 ( .A(n780), .B(n306), .C(n259), .D(n290), .Z(n493) );
  HS65_GS_IVX9 U224 ( .A(inst_bw), .Z(n785) );
  HS65_GS_NAND2X7 U225 ( .A(inst_bw), .B(n472), .Z(incr_op_1_) );
  HS65_GS_FA1X4 U226 ( .A0(reg_src[1]), .B0(incr_op_1_), .CI(add_155_carry[1]), 
        .CO(add_155_carry[2]), .S0(reg_incr_val[1]) );
  HS65_GS_NOR4ABX4 U227 ( .A(reg_incr), .B(n778), .C(n781), .D(reg_sp_wr), .Z(
        n468) );
  HS65_GS_NAND2X7 U228 ( .A(reg_dest_wr), .B(inst_dest[10]), .Z(n390) );
  HS65_GS_NAND2X7 U229 ( .A(reg_dest_wr), .B(inst_dest[11]), .Z(n371) );
  HS65_GS_NAND2X7 U230 ( .A(reg_dest_wr), .B(inst_dest[12]), .Z(n352) );
  HS65_GS_NAND2X7 U231 ( .A(reg_dest_wr), .B(inst_dest[13]), .Z(n333) );
  HS65_GS_NAND2X7 U232 ( .A(reg_dest_wr), .B(inst_dest[14]), .Z(n312) );
  HS65_GS_AOI12X2 U233 ( .A(reg_dest_wr), .B(inst_dest[2]), .C(reg_sr_wr), .Z(
        n478) );
  HS65_GS_NAND2X7 U234 ( .A(reg_dest_wr), .B(inst_dest[5]), .Z(n451) );
  HS65_GS_NAND2X7 U235 ( .A(reg_dest_wr), .B(inst_dest[6]), .Z(n435) );
  HS65_GS_NAND2X7 U236 ( .A(reg_dest_wr), .B(inst_dest[7]), .Z(n429) );
  HS65_GS_NAND2X7 U237 ( .A(reg_dest_wr), .B(inst_dest[8]), .Z(n426) );
  HS65_GS_NAND2X7 U238 ( .A(reg_dest_wr), .B(inst_dest[9]), .Z(n409) );
  HS65_GS_NAND2X7 U239 ( .A(reg_dest_wr), .B(inst_dest[4]), .Z(n464) );
  HS65_GS_NAND2X7 U240 ( .A(reg_dest_wr), .B(inst_dest[15]), .Z(n277) );
  HS65_GS_NAND2X7 U241 ( .A(reg_dest_wr), .B(inst_dest[3]), .Z(n259) );
  HS65_GSS_XNOR2X6 U242 ( .A(reg_src[0]), .B(add_155_B_0_), .Z(n133) );
  HS65_GS_NAND3X5 U243 ( .A(n22), .B(n312), .C(reg_incr), .Z(n309) );
  HS65_GS_NAND3X5 U244 ( .A(n18), .B(n390), .C(reg_incr), .Z(n387) );
  HS65_GS_NAND3X5 U245 ( .A(n19), .B(n371), .C(reg_incr), .Z(n368) );
  HS65_GS_NAND3X5 U246 ( .A(n20), .B(n352), .C(reg_incr), .Z(n349) );
  HS65_GS_NAND3X5 U247 ( .A(n21), .B(n333), .C(reg_incr), .Z(n330) );
  HS65_GS_NAND2X7 U248 ( .A(reg_dest_wr), .B(inst_dest[1]), .Z(n473) );
  HS65_GS_NAND3X5 U249 ( .A(n252), .B(n277), .C(reg_incr), .Z(n278) );
  HS65_GS_NAND3X5 U250 ( .A(n777), .B(n464), .C(reg_incr), .Z(n465) );
  HS65_GS_NAND3X5 U251 ( .A(n776), .B(n451), .C(reg_incr), .Z(n448) );
  HS65_GS_NAND3X5 U252 ( .A(n775), .B(n435), .C(reg_incr), .Z(n432) );
  HS65_GS_NAND3X5 U253 ( .A(n774), .B(n429), .C(reg_incr), .Z(n427) );
  HS65_GS_NAND3X5 U254 ( .A(n773), .B(n426), .C(reg_incr), .Z(n424) );
  HS65_GS_NAND3X5 U255 ( .A(n772), .B(n409), .C(reg_incr), .Z(n406) );
  HS65_GS_AND2X4 U256 ( .A(reg_sp_wr), .B(n473), .Z(n467) );
  HS65_GS_IVX9 U257 ( .A(puc_rst), .Z(n771) );
  HS65_GS_OAI222X2 U258 ( .A(n759), .B(n276), .C(n274), .D(n277), .E(n278), 
        .F(n253), .Z(n509) );
  HS65_GS_IVX9 U259 ( .A(OBSERVE_r15[15]), .Z(n759) );
  HS65_GS_OAI222X2 U260 ( .A(n747), .B(n276), .C(n273), .D(n277), .E(n278), 
        .F(n260), .Z(n510) );
  HS65_GS_IVX9 U261 ( .A(OBSERVE_r15[14]), .Z(n747) );
  HS65_GS_OAI222X2 U262 ( .A(n735), .B(n276), .C(n272), .D(n277), .E(n278), 
        .F(n261), .Z(n511) );
  HS65_GS_IVX9 U263 ( .A(OBSERVE_r15[13]), .Z(n735) );
  HS65_GS_OAI222X2 U264 ( .A(n253), .B(n309), .C(n760), .D(n311), .E(n274), 
        .F(n312), .Z(n524) );
  HS65_GS_IVX9 U265 ( .A(OBSERVE_r14[15]), .Z(n760) );
  HS65_GS_OAI222X2 U266 ( .A(n260), .B(n309), .C(n748), .D(n311), .E(n273), 
        .F(n312), .Z(n525) );
  HS65_GS_IVX9 U267 ( .A(OBSERVE_r14[14]), .Z(n748) );
  HS65_GS_OAI222X2 U268 ( .A(n261), .B(n309), .C(n736), .D(n311), .E(n272), 
        .F(n312), .Z(n526) );
  HS65_GS_IVX9 U269 ( .A(OBSERVE_r14[13]), .Z(n736) );
  HS65_GS_OAI222X2 U270 ( .A(n253), .B(n387), .C(n764), .D(n389), .E(n274), 
        .F(n390), .Z(n588) );
  HS65_GS_IVX9 U271 ( .A(OBSERVE_r10[15]), .Z(n764) );
  HS65_GS_OAI222X2 U272 ( .A(n253), .B(n368), .C(n763), .D(n370), .E(n274), 
        .F(n371), .Z(n572) );
  HS65_GS_IVX9 U273 ( .A(OBSERVE_r11[15]), .Z(n763) );
  HS65_GS_OAI222X2 U274 ( .A(n253), .B(n349), .C(n762), .D(n351), .E(n274), 
        .F(n352), .Z(n556) );
  HS65_GS_IVX9 U275 ( .A(OBSERVE_r12[15]), .Z(n762) );
  HS65_GS_OAI222X2 U276 ( .A(n253), .B(n330), .C(n761), .D(n332), .E(n274), 
        .F(n333), .Z(n540) );
  HS65_GS_IVX9 U277 ( .A(OBSERVE_r13[15]), .Z(n761) );
  HS65_GS_OAI222X2 U278 ( .A(n260), .B(n387), .C(n752), .D(n389), .E(n273), 
        .F(n390), .Z(n589) );
  HS65_GS_IVX9 U279 ( .A(OBSERVE_r10[14]), .Z(n752) );
  HS65_GS_OAI222X2 U280 ( .A(n260), .B(n368), .C(n751), .D(n370), .E(n273), 
        .F(n371), .Z(n573) );
  HS65_GS_IVX9 U281 ( .A(OBSERVE_r11[14]), .Z(n751) );
  HS65_GS_OAI222X2 U282 ( .A(n260), .B(n349), .C(n750), .D(n351), .E(n273), 
        .F(n352), .Z(n557) );
  HS65_GS_IVX9 U283 ( .A(OBSERVE_r12[14]), .Z(n750) );
  HS65_GS_OAI222X2 U284 ( .A(n260), .B(n330), .C(n749), .D(n332), .E(n273), 
        .F(n333), .Z(n541) );
  HS65_GS_IVX9 U285 ( .A(OBSERVE_r13[14]), .Z(n749) );
  HS65_GS_OAI222X2 U286 ( .A(n261), .B(n387), .C(n740), .D(n389), .E(n272), 
        .F(n390), .Z(n590) );
  HS65_GS_IVX9 U287 ( .A(OBSERVE_r10[13]), .Z(n740) );
  HS65_GS_OAI222X2 U288 ( .A(n261), .B(n368), .C(n739), .D(n370), .E(n272), 
        .F(n371), .Z(n574) );
  HS65_GS_IVX9 U289 ( .A(OBSERVE_r11[13]), .Z(n739) );
  HS65_GS_OAI222X2 U290 ( .A(n261), .B(n349), .C(n738), .D(n351), .E(n272), 
        .F(n352), .Z(n558) );
  HS65_GS_IVX9 U291 ( .A(OBSERVE_r12[13]), .Z(n738) );
  HS65_GS_OAI222X2 U292 ( .A(n261), .B(n330), .C(n737), .D(n332), .E(n272), 
        .F(n333), .Z(n542) );
  HS65_GS_IVX9 U293 ( .A(OBSERVE_r13[13]), .Z(n737) );
  HS65_GS_OAI222X2 U294 ( .A(n253), .B(n448), .C(n769), .D(n450), .E(n274), 
        .F(n451), .Z(n668) );
  HS65_GS_IVX9 U295 ( .A(OBSERVE_r5[15]), .Z(n769) );
  HS65_GS_OAI222X2 U296 ( .A(n253), .B(n432), .C(n768), .D(n434), .E(n274), 
        .F(n435), .Z(n652) );
  HS65_GS_IVX9 U297 ( .A(OBSERVE_r6[15]), .Z(n768) );
  HS65_GS_OAI222X2 U298 ( .A(n253), .B(n406), .C(n765), .D(n408), .E(n274), 
        .F(n409), .Z(n604) );
  HS65_GS_IVX9 U299 ( .A(OBSERVE_r9[15]), .Z(n765) );
  HS65_GS_OAI222X2 U300 ( .A(n260), .B(n448), .C(n757), .D(n450), .E(n273), 
        .F(n451), .Z(n669) );
  HS65_GS_IVX9 U301 ( .A(OBSERVE_r5[14]), .Z(n757) );
  HS65_GS_OAI222X2 U302 ( .A(n260), .B(n432), .C(n756), .D(n434), .E(n273), 
        .F(n435), .Z(n653) );
  HS65_GS_IVX9 U303 ( .A(OBSERVE_r6[14]), .Z(n756) );
  HS65_GS_OAI222X2 U304 ( .A(n260), .B(n406), .C(n753), .D(n408), .E(n273), 
        .F(n409), .Z(n605) );
  HS65_GS_IVX9 U305 ( .A(OBSERVE_r9[14]), .Z(n753) );
  HS65_GS_OAI222X2 U306 ( .A(n261), .B(n448), .C(n745), .D(n450), .E(n272), 
        .F(n451), .Z(n670) );
  HS65_GS_IVX9 U307 ( .A(OBSERVE_r5[13]), .Z(n745) );
  HS65_GS_OAI222X2 U308 ( .A(n261), .B(n432), .C(n744), .D(n434), .E(n272), 
        .F(n435), .Z(n654) );
  HS65_GS_IVX9 U309 ( .A(OBSERVE_r6[13]), .Z(n744) );
  HS65_GS_OAI222X2 U310 ( .A(n261), .B(n406), .C(n741), .D(n408), .E(n272), 
        .F(n409), .Z(n606) );
  HS65_GS_IVX9 U311 ( .A(OBSERVE_r9[13]), .Z(n741) );
  HS65_GS_IVX9 U312 ( .A(n273), .Z(pc_sw[14]) );
  HS65_GS_CBI4I6X5 U313 ( .A(alu_stat_wr[0]), .B(n485), .C(n486), .D(
        reg_sr_clr), .Z(N76) );
  HS65_GS_AOI22X6 U314 ( .A(reg_dest_val[0]), .B(n782), .C(n478), .D(status[0]), .Z(n485) );
  HS65_GS_NAND2X7 U315 ( .A(alu_stat_wr[0]), .B(alu_stat[0]), .Z(n486) );
  HS65_GS_CBI4I6X5 U316 ( .A(alu_stat_wr[1]), .B(n483), .C(n484), .D(
        reg_sr_clr), .Z(N77) );
  HS65_GS_AOI22X6 U317 ( .A(reg_dest_val[1]), .B(n782), .C(n478), .D(status[1]), .Z(n483) );
  HS65_GS_NAND2X7 U318 ( .A(alu_stat_wr[1]), .B(alu_stat[1]), .Z(n484) );
  HS65_GS_CBI4I6X5 U319 ( .A(alu_stat_wr[2]), .B(n481), .C(n482), .D(
        reg_sr_clr), .Z(N78) );
  HS65_GS_AOI22X6 U320 ( .A(reg_dest_val[2]), .B(n782), .C(n478), .D(status[2]), .Z(n481) );
  HS65_GS_NAND2X7 U321 ( .A(alu_stat_wr[2]), .B(alu_stat[2]), .Z(n482) );
  HS65_GS_CBI4I6X5 U322 ( .A(alu_stat_wr[3]), .B(n475), .C(n476), .D(
        reg_sr_clr), .Z(N84) );
  HS65_GS_AOI22X6 U323 ( .A(pc_sw[8]), .B(n782), .C(n478), .D(status[3]), .Z(
        n475) );
  HS65_GS_NAND2X7 U324 ( .A(alu_stat_wr[3]), .B(alu_stat[3]), .Z(n476) );
  HS65_GS_IVX9 U325 ( .A(n271), .Z(pc_sw[12]) );
  HS65_GS_IVX9 U326 ( .A(n270), .Z(pc_sw[11]) );
  HS65_GS_IVX9 U327 ( .A(n269), .Z(pc_sw[10]) );
  HS65_GS_IVX9 U328 ( .A(n268), .Z(pc_sw[9]) );
  HS65_GS_IVX9 U329 ( .A(n267), .Z(pc_sw[8]) );
  HS65_GS_MX41X7 U330 ( .D0(reg_sp_val[15]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[15]), .D2(n469), .S2(OBSERVE_r1[15]), .D3(n781), .S3(
        pc_sw[15]), .Z(n700) );
  HS65_GS_MX41X7 U331 ( .D0(reg_sp_val[14]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[14]), .D2(n469), .S2(OBSERVE_r1[14]), .D3(n781), .S3(
        pc_sw[14]), .Z(n701) );
  HS65_GS_MX41X7 U332 ( .D0(reg_sp_val[13]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[13]), .D2(n469), .S2(OBSERVE_r1[13]), .D3(n781), .S3(
        pc_sw[13]), .Z(n702) );
  HS65_GS_IVX9 U333 ( .A(n272), .Z(pc_sw[13]) );
  HS65_GS_OAI222X2 U334 ( .A(n723), .B(n276), .C(n271), .D(n277), .E(n278), 
        .F(n262), .Z(n512) );
  HS65_GS_IVX9 U335 ( .A(OBSERVE_r15[12]), .Z(n723) );
  HS65_GS_OAI222X2 U336 ( .A(n474), .B(n276), .C(n270), .D(n277), .E(n278), 
        .F(n263), .Z(n513) );
  HS65_GS_IVX9 U337 ( .A(OBSERVE_r15[11]), .Z(n474) );
  HS65_GS_OAI222X2 U338 ( .A(n454), .B(n276), .C(n269), .D(n277), .E(n278), 
        .F(n264), .Z(n514) );
  HS65_GS_IVX9 U339 ( .A(OBSERVE_r15[10]), .Z(n454) );
  HS65_GS_OAI222X2 U340 ( .A(n439), .B(n276), .C(n268), .D(n277), .E(n278), 
        .F(n265), .Z(n515) );
  HS65_GS_IVX9 U341 ( .A(OBSERVE_r15[9]), .Z(n439) );
  HS65_GS_OAI222X2 U342 ( .A(n419), .B(n276), .C(n267), .D(n277), .E(n278), 
        .F(n266), .Z(n516) );
  HS65_GS_IVX9 U343 ( .A(OBSERVE_r15[8]), .Z(n419) );
  HS65_GS_AOI222X2 U344 ( .A(OBSERVE_r1[4]), .B(n778), .C(OBSERVE_r4[4]), .D(
        n777), .E(OBSERVE_r15[4]), .F(n252), .Z(n63) );
  HS65_GS_AOI222X2 U345 ( .A(OBSERVE_r1[5]), .B(n778), .C(OBSERVE_r4[5]), .D(
        n777), .E(OBSERVE_r15[5]), .F(n252), .Z(n56) );
  HS65_GS_AOI222X2 U346 ( .A(OBSERVE_r1[6]), .B(n778), .C(OBSERVE_r4[6]), .D(
        n777), .E(OBSERVE_r15[6]), .F(n252), .Z(n49) );
  HS65_GS_AOI222X2 U347 ( .A(OBSERVE_r1[7]), .B(n778), .C(OBSERVE_r4[7]), .D(
        n777), .E(OBSERVE_r15[7]), .F(n252), .Z(n42) );
  HS65_GS_AOI222X2 U348 ( .A(OBSERVE_r1[9]), .B(n778), .C(OBSERVE_r4[9]), .D(
        n777), .E(OBSERVE_r15[9]), .F(n252), .Z(n5) );
  HS65_GS_AOI222X2 U349 ( .A(OBSERVE_r1[15]), .B(n778), .C(OBSERVE_r4[15]), 
        .D(n777), .E(OBSERVE_r15[15]), .F(n252), .Z(n102) );
  HS65_GS_AOI222X2 U350 ( .A(OBSERVE_r1[10]), .B(n778), .C(OBSERVE_r4[10]), 
        .D(n777), .E(OBSERVE_r15[10]), .F(n252), .Z(n137) );
  HS65_GS_AOI222X2 U351 ( .A(OBSERVE_r1[11]), .B(n778), .C(OBSERVE_r4[11]), 
        .D(n777), .E(OBSERVE_r15[11]), .F(n252), .Z(n130) );
  HS65_GS_AOI222X2 U352 ( .A(OBSERVE_r1[12]), .B(n778), .C(OBSERVE_r4[12]), 
        .D(n777), .E(OBSERVE_r15[12]), .F(n252), .Z(n123) );
  HS65_GS_AOI222X2 U353 ( .A(OBSERVE_r1[13]), .B(n778), .C(OBSERVE_r4[13]), 
        .D(n777), .E(OBSERVE_r15[13]), .F(n252), .Z(n116) );
  HS65_GS_AOI222X2 U354 ( .A(OBSERVE_r1[14]), .B(n778), .C(OBSERVE_r4[14]), 
        .D(n777), .E(OBSERVE_r15[14]), .F(n252), .Z(n109) );
  HS65_GS_OAI222X2 U355 ( .A(n262), .B(n309), .C(n724), .D(n311), .E(n271), 
        .F(n312), .Z(n527) );
  HS65_GS_IVX9 U356 ( .A(OBSERVE_r14[12]), .Z(n724) );
  HS65_GS_OAI222X2 U357 ( .A(n263), .B(n309), .C(n477), .D(n311), .E(n270), 
        .F(n312), .Z(n528) );
  HS65_GS_IVX9 U358 ( .A(OBSERVE_r14[11]), .Z(n477) );
  HS65_GS_OAI222X2 U359 ( .A(n264), .B(n309), .C(n455), .D(n311), .E(n269), 
        .F(n312), .Z(n529) );
  HS65_GS_IVX9 U360 ( .A(OBSERVE_r14[10]), .Z(n455) );
  HS65_GS_OAI222X2 U361 ( .A(n265), .B(n309), .C(n440), .D(n311), .E(n268), 
        .F(n312), .Z(n530) );
  HS65_GS_IVX9 U362 ( .A(OBSERVE_r14[9]), .Z(n440) );
  HS65_GS_OAI222X2 U363 ( .A(n266), .B(n309), .C(n420), .D(n311), .E(n267), 
        .F(n312), .Z(n531) );
  HS65_GS_IVX9 U364 ( .A(OBSERVE_r14[8]), .Z(n420) );
  HS65_GS_OAI222X2 U365 ( .A(n275), .B(n309), .C(n405), .D(n311), .E(n285), 
        .F(n312), .Z(n532) );
  HS65_GS_IVX9 U366 ( .A(OBSERVE_r14[7]), .Z(n405) );
  HS65_GS_OAI222X2 U367 ( .A(n279), .B(n309), .C(n393), .D(n311), .E(n286), 
        .F(n312), .Z(n533) );
  HS65_GS_IVX9 U368 ( .A(OBSERVE_r14[6]), .Z(n393) );
  HS65_GS_OAI222X2 U369 ( .A(n280), .B(n309), .C(n378), .D(n311), .E(n287), 
        .F(n312), .Z(n534) );
  HS65_GS_IVX9 U370 ( .A(OBSERVE_r14[5]), .Z(n378) );
  HS65_GS_OAI222X2 U371 ( .A(n281), .B(n309), .C(n363), .D(n311), .E(n288), 
        .F(n312), .Z(n535) );
  HS65_GS_IVX9 U372 ( .A(OBSERVE_r14[4]), .Z(n363) );
  HS65_GS_OAI222X2 U373 ( .A(n282), .B(n309), .C(n348), .D(n311), .E(n289), 
        .F(n312), .Z(n536) );
  HS65_GS_IVX9 U374 ( .A(OBSERVE_r14[3]), .Z(n348) );
  HS65_GS_OAI222X2 U375 ( .A(n262), .B(n387), .C(n728), .D(n389), .E(n271), 
        .F(n390), .Z(n591) );
  HS65_GS_IVX9 U376 ( .A(OBSERVE_r10[12]), .Z(n728) );
  HS65_GS_OAI222X2 U377 ( .A(n262), .B(n368), .C(n727), .D(n370), .E(n271), 
        .F(n371), .Z(n575) );
  HS65_GS_IVX9 U378 ( .A(OBSERVE_r11[12]), .Z(n727) );
  HS65_GS_OAI222X2 U379 ( .A(n262), .B(n349), .C(n726), .D(n351), .E(n271), 
        .F(n352), .Z(n559) );
  HS65_GS_IVX9 U380 ( .A(OBSERVE_r12[12]), .Z(n726) );
  HS65_GS_OAI222X2 U381 ( .A(n262), .B(n330), .C(n725), .D(n332), .E(n271), 
        .F(n333), .Z(n543) );
  HS65_GS_IVX9 U382 ( .A(OBSERVE_r13[12]), .Z(n725) );
  HS65_GS_OAI222X2 U383 ( .A(n263), .B(n387), .C(n716), .D(n389), .E(n270), 
        .F(n390), .Z(n592) );
  HS65_GS_IVX9 U384 ( .A(OBSERVE_r10[11]), .Z(n716) );
  HS65_GS_OAI222X2 U385 ( .A(n263), .B(n368), .C(n508), .D(n370), .E(n270), 
        .F(n371), .Z(n576) );
  HS65_GS_IVX9 U386 ( .A(OBSERVE_r11[11]), .Z(n508) );
  HS65_GS_OAI222X2 U387 ( .A(n263), .B(n349), .C(n507), .D(n351), .E(n270), 
        .F(n352), .Z(n560) );
  HS65_GS_IVX9 U388 ( .A(OBSERVE_r12[11]), .Z(n507) );
  HS65_GS_OAI222X2 U389 ( .A(n263), .B(n330), .C(n479), .D(n332), .E(n270), 
        .F(n333), .Z(n544) );
  HS65_GS_IVX9 U390 ( .A(OBSERVE_r13[11]), .Z(n479) );
  HS65_GS_OAI222X2 U391 ( .A(n264), .B(n387), .C(n459), .D(n389), .E(n269), 
        .F(n390), .Z(n593) );
  HS65_GS_IVX9 U392 ( .A(OBSERVE_r10[10]), .Z(n459) );
  HS65_GS_OAI222X2 U393 ( .A(n264), .B(n368), .C(n458), .D(n370), .E(n269), 
        .F(n371), .Z(n577) );
  HS65_GS_IVX9 U394 ( .A(OBSERVE_r11[10]), .Z(n458) );
  HS65_GS_OAI222X2 U395 ( .A(n264), .B(n349), .C(n457), .D(n351), .E(n269), 
        .F(n352), .Z(n561) );
  HS65_GS_IVX9 U396 ( .A(OBSERVE_r12[10]), .Z(n457) );
  HS65_GS_OAI222X2 U397 ( .A(n264), .B(n330), .C(n456), .D(n332), .E(n269), 
        .F(n333), .Z(n545) );
  HS65_GS_IVX9 U398 ( .A(OBSERVE_r13[10]), .Z(n456) );
  HS65_GS_OAI222X2 U399 ( .A(n265), .B(n387), .C(n444), .D(n389), .E(n268), 
        .F(n390), .Z(n594) );
  HS65_GS_IVX9 U400 ( .A(OBSERVE_r10[9]), .Z(n444) );
  HS65_GS_OAI222X2 U401 ( .A(n265), .B(n368), .C(n443), .D(n370), .E(n268), 
        .F(n371), .Z(n578) );
  HS65_GS_IVX9 U402 ( .A(OBSERVE_r11[9]), .Z(n443) );
  HS65_GS_OAI222X2 U403 ( .A(n265), .B(n349), .C(n442), .D(n351), .E(n268), 
        .F(n352), .Z(n562) );
  HS65_GS_IVX9 U404 ( .A(OBSERVE_r12[9]), .Z(n442) );
  HS65_GS_OAI222X2 U405 ( .A(n265), .B(n330), .C(n441), .D(n332), .E(n268), 
        .F(n333), .Z(n546) );
  HS65_GS_IVX9 U406 ( .A(OBSERVE_r13[9]), .Z(n441) );
  HS65_GS_OAI222X2 U407 ( .A(n266), .B(n387), .C(n430), .D(n389), .E(n267), 
        .F(n390), .Z(n595) );
  HS65_GS_IVX9 U408 ( .A(OBSERVE_r10[8]), .Z(n430) );
  HS65_GS_OAI222X2 U409 ( .A(n266), .B(n368), .C(n423), .D(n370), .E(n267), 
        .F(n371), .Z(n579) );
  HS65_GS_IVX9 U410 ( .A(OBSERVE_r11[8]), .Z(n423) );
  HS65_GS_OAI222X2 U411 ( .A(n266), .B(n349), .C(n422), .D(n351), .E(n267), 
        .F(n352), .Z(n563) );
  HS65_GS_IVX9 U412 ( .A(OBSERVE_r12[8]), .Z(n422) );
  HS65_GS_OAI222X2 U413 ( .A(n266), .B(n330), .C(n421), .D(n332), .E(n267), 
        .F(n333), .Z(n547) );
  HS65_GS_IVX9 U414 ( .A(OBSERVE_r13[8]), .Z(n421) );
  HS65_GS_OAI222X2 U415 ( .A(n275), .B(n387), .C(n412), .D(n389), .E(n285), 
        .F(n390), .Z(n596) );
  HS65_GS_IVX9 U416 ( .A(OBSERVE_r10[7]), .Z(n412) );
  HS65_GS_OAI222X2 U417 ( .A(n275), .B(n368), .C(n411), .D(n370), .E(n285), 
        .F(n371), .Z(n580) );
  HS65_GS_IVX9 U418 ( .A(OBSERVE_r11[7]), .Z(n411) );
  HS65_GS_OAI222X2 U419 ( .A(n275), .B(n349), .C(n410), .D(n351), .E(n285), 
        .F(n352), .Z(n564) );
  HS65_GS_IVX9 U420 ( .A(OBSERVE_r12[7]), .Z(n410) );
  HS65_GS_OAI222X2 U421 ( .A(n275), .B(n330), .C(n407), .D(n332), .E(n285), 
        .F(n333), .Z(n548) );
  HS65_GS_IVX9 U422 ( .A(OBSERVE_r13[7]), .Z(n407) );
  HS65_GS_OAI222X2 U423 ( .A(n279), .B(n387), .C(n397), .D(n389), .E(n286), 
        .F(n390), .Z(n597) );
  HS65_GS_IVX9 U424 ( .A(OBSERVE_r10[6]), .Z(n397) );
  HS65_GS_OAI222X2 U425 ( .A(n279), .B(n368), .C(n396), .D(n370), .E(n286), 
        .F(n371), .Z(n581) );
  HS65_GS_IVX9 U426 ( .A(OBSERVE_r11[6]), .Z(n396) );
  HS65_GS_OAI222X2 U427 ( .A(n279), .B(n349), .C(n395), .D(n351), .E(n286), 
        .F(n352), .Z(n565) );
  HS65_GS_IVX9 U428 ( .A(OBSERVE_r12[6]), .Z(n395) );
  HS65_GS_OAI222X2 U429 ( .A(n279), .B(n330), .C(n394), .D(n332), .E(n286), 
        .F(n333), .Z(n549) );
  HS65_GS_IVX9 U430 ( .A(OBSERVE_r13[6]), .Z(n394) );
  HS65_GS_OAI222X2 U431 ( .A(n280), .B(n387), .C(n382), .D(n389), .E(n287), 
        .F(n390), .Z(n598) );
  HS65_GS_IVX9 U432 ( .A(OBSERVE_r10[5]), .Z(n382) );
  HS65_GS_OAI222X2 U433 ( .A(n280), .B(n368), .C(n381), .D(n370), .E(n287), 
        .F(n371), .Z(n582) );
  HS65_GS_IVX9 U434 ( .A(OBSERVE_r11[5]), .Z(n381) );
  HS65_GS_OAI222X2 U435 ( .A(n280), .B(n349), .C(n380), .D(n351), .E(n287), 
        .F(n352), .Z(n566) );
  HS65_GS_IVX9 U436 ( .A(OBSERVE_r12[5]), .Z(n380) );
  HS65_GS_OAI222X2 U437 ( .A(n280), .B(n330), .C(n379), .D(n332), .E(n287), 
        .F(n333), .Z(n550) );
  HS65_GS_IVX9 U438 ( .A(OBSERVE_r13[5]), .Z(n379) );
  HS65_GS_OAI222X2 U439 ( .A(n281), .B(n387), .C(n367), .D(n389), .E(n288), 
        .F(n390), .Z(n599) );
  HS65_GS_IVX9 U440 ( .A(OBSERVE_r10[4]), .Z(n367) );
  HS65_GS_OAI222X2 U441 ( .A(n281), .B(n368), .C(n366), .D(n370), .E(n288), 
        .F(n371), .Z(n583) );
  HS65_GS_IVX9 U442 ( .A(OBSERVE_r11[4]), .Z(n366) );
  HS65_GS_OAI222X2 U443 ( .A(n281), .B(n349), .C(n365), .D(n351), .E(n288), 
        .F(n352), .Z(n567) );
  HS65_GS_IVX9 U444 ( .A(OBSERVE_r12[4]), .Z(n365) );
  HS65_GS_OAI222X2 U445 ( .A(n281), .B(n330), .C(n364), .D(n332), .E(n288), 
        .F(n333), .Z(n551) );
  HS65_GS_IVX9 U446 ( .A(OBSERVE_r13[4]), .Z(n364) );
  HS65_GS_OAI222X2 U447 ( .A(n282), .B(n387), .C(n355), .D(n389), .E(n289), 
        .F(n390), .Z(n600) );
  HS65_GS_IVX9 U448 ( .A(OBSERVE_r10[3]), .Z(n355) );
  HS65_GS_OAI222X2 U449 ( .A(n282), .B(n368), .C(n354), .D(n370), .E(n289), 
        .F(n371), .Z(n584) );
  HS65_GS_IVX9 U450 ( .A(OBSERVE_r11[3]), .Z(n354) );
  HS65_GS_OAI222X2 U451 ( .A(n282), .B(n349), .C(n353), .D(n351), .E(n289), 
        .F(n352), .Z(n568) );
  HS65_GS_IVX9 U452 ( .A(OBSERVE_r12[3]), .Z(n353) );
  HS65_GS_OAI222X2 U453 ( .A(n282), .B(n330), .C(n350), .D(n332), .E(n289), 
        .F(n333), .Z(n552) );
  HS65_GS_IVX9 U454 ( .A(OBSERVE_r13[3]), .Z(n350) );
  HS65_GS_OAI222X2 U455 ( .A(n278), .B(n275), .C(n404), .D(n276), .E(n285), 
        .F(n277), .Z(n517) );
  HS65_GS_IVX9 U456 ( .A(OBSERVE_r15[7]), .Z(n404) );
  HS65_GS_OAI222X2 U457 ( .A(n278), .B(n279), .C(n392), .D(n276), .E(n286), 
        .F(n277), .Z(n518) );
  HS65_GS_IVX9 U458 ( .A(OBSERVE_r15[6]), .Z(n392) );
  HS65_GS_OAI222X2 U459 ( .A(n278), .B(n280), .C(n377), .D(n276), .E(n287), 
        .F(n277), .Z(n519) );
  HS65_GS_IVX9 U460 ( .A(OBSERVE_r15[5]), .Z(n377) );
  HS65_GS_OAI222X2 U461 ( .A(n278), .B(n281), .C(n362), .D(n276), .E(n288), 
        .F(n277), .Z(n520) );
  HS65_GS_IVX9 U462 ( .A(OBSERVE_r15[4]), .Z(n362) );
  HS65_GS_OAI222X2 U463 ( .A(n278), .B(n282), .C(n347), .D(n276), .E(n289), 
        .F(n277), .Z(n521) );
  HS65_GS_IVX9 U464 ( .A(OBSERVE_r15[3]), .Z(n347) );
  HS65_GS_OAI222X2 U465 ( .A(n266), .B(n424), .C(n487), .D(n425), .E(n267), 
        .F(n426), .Z(n627) );
  HS65_GS_OAI222X2 U466 ( .A(n262), .B(n448), .C(n733), .D(n450), .E(n271), 
        .F(n451), .Z(n671) );
  HS65_GS_IVX9 U467 ( .A(OBSERVE_r5[12]), .Z(n733) );
  HS65_GS_OAI222X2 U468 ( .A(n262), .B(n432), .C(n732), .D(n434), .E(n271), 
        .F(n435), .Z(n655) );
  HS65_GS_IVX9 U469 ( .A(OBSERVE_r6[12]), .Z(n732) );
  HS65_GS_OAI222X2 U470 ( .A(n262), .B(n406), .C(n729), .D(n408), .E(n271), 
        .F(n409), .Z(n607) );
  HS65_GS_IVX9 U471 ( .A(OBSERVE_r9[12]), .Z(n729) );
  HS65_GS_OAI222X2 U472 ( .A(n263), .B(n448), .C(n721), .D(n450), .E(n270), 
        .F(n451), .Z(n672) );
  HS65_GS_IVX9 U473 ( .A(OBSERVE_r5[11]), .Z(n721) );
  HS65_GS_OAI222X2 U474 ( .A(n263), .B(n432), .C(n720), .D(n434), .E(n270), 
        .F(n435), .Z(n656) );
  HS65_GS_IVX9 U475 ( .A(OBSERVE_r6[11]), .Z(n720) );
  HS65_GS_OAI222X2 U476 ( .A(n263), .B(n406), .C(n717), .D(n408), .E(n270), 
        .F(n409), .Z(n608) );
  HS65_GS_IVX9 U477 ( .A(OBSERVE_r9[11]), .Z(n717) );
  HS65_GS_OAI222X2 U478 ( .A(n264), .B(n448), .C(n470), .D(n450), .E(n269), 
        .F(n451), .Z(n673) );
  HS65_GS_IVX9 U479 ( .A(OBSERVE_r5[10]), .Z(n470) );
  HS65_GS_OAI222X2 U480 ( .A(n264), .B(n432), .C(n466), .D(n434), .E(n269), 
        .F(n435), .Z(n657) );
  HS65_GS_IVX9 U481 ( .A(OBSERVE_r6[10]), .Z(n466) );
  HS65_GS_OAI222X2 U482 ( .A(n264), .B(n406), .C(n460), .D(n408), .E(n269), 
        .F(n409), .Z(n609) );
  HS65_GS_IVX9 U483 ( .A(OBSERVE_r9[10]), .Z(n460) );
  HS65_GS_OAI222X2 U484 ( .A(n265), .B(n448), .C(n452), .D(n450), .E(n268), 
        .F(n451), .Z(n674) );
  HS65_GS_IVX9 U485 ( .A(OBSERVE_r5[9]), .Z(n452) );
  HS65_GS_OAI222X2 U486 ( .A(n265), .B(n432), .C(n449), .D(n434), .E(n268), 
        .F(n435), .Z(n658) );
  HS65_GS_IVX9 U487 ( .A(OBSERVE_r6[9]), .Z(n449) );
  HS65_GS_OAI222X2 U488 ( .A(n265), .B(n406), .C(n445), .D(n408), .E(n268), 
        .F(n409), .Z(n610) );
  HS65_GS_IVX9 U489 ( .A(OBSERVE_r9[9]), .Z(n445) );
  HS65_GS_OAI222X2 U490 ( .A(n266), .B(n406), .C(n431), .D(n408), .E(n267), 
        .F(n409), .Z(n611) );
  HS65_GS_IVX9 U491 ( .A(OBSERVE_r9[8]), .Z(n431) );
  HS65_GS_OAI222X2 U492 ( .A(n275), .B(n448), .C(n417), .D(n450), .E(n285), 
        .F(n451), .Z(n676) );
  HS65_GS_IVX9 U493 ( .A(OBSERVE_r5[7]), .Z(n417) );
  HS65_GS_OAI222X2 U494 ( .A(n275), .B(n432), .C(n416), .D(n434), .E(n285), 
        .F(n435), .Z(n660) );
  HS65_GS_IVX9 U495 ( .A(OBSERVE_r6[7]), .Z(n416) );
  HS65_GS_OAI222X2 U496 ( .A(n275), .B(n406), .C(n413), .D(n408), .E(n285), 
        .F(n409), .Z(n612) );
  HS65_GS_IVX9 U497 ( .A(OBSERVE_r9[7]), .Z(n413) );
  HS65_GS_OAI222X2 U498 ( .A(n279), .B(n448), .C(n402), .D(n450), .E(n286), 
        .F(n451), .Z(n677) );
  HS65_GS_IVX9 U499 ( .A(OBSERVE_r5[6]), .Z(n402) );
  HS65_GS_OAI222X2 U500 ( .A(n279), .B(n432), .C(n401), .D(n434), .E(n286), 
        .F(n435), .Z(n661) );
  HS65_GS_IVX9 U501 ( .A(OBSERVE_r6[6]), .Z(n401) );
  HS65_GS_OAI222X2 U502 ( .A(n279), .B(n406), .C(n398), .D(n408), .E(n286), 
        .F(n409), .Z(n613) );
  HS65_GS_IVX9 U503 ( .A(OBSERVE_r9[6]), .Z(n398) );
  HS65_GS_OAI222X2 U504 ( .A(n280), .B(n448), .C(n388), .D(n450), .E(n287), 
        .F(n451), .Z(n678) );
  HS65_GS_IVX9 U505 ( .A(OBSERVE_r5[5]), .Z(n388) );
  HS65_GS_OAI222X2 U506 ( .A(n280), .B(n432), .C(n386), .D(n434), .E(n287), 
        .F(n435), .Z(n662) );
  HS65_GS_IVX9 U507 ( .A(OBSERVE_r6[5]), .Z(n386) );
  HS65_GS_OAI222X2 U508 ( .A(n280), .B(n406), .C(n383), .D(n408), .E(n287), 
        .F(n409), .Z(n614) );
  HS65_GS_IVX9 U509 ( .A(OBSERVE_r9[5]), .Z(n383) );
  HS65_GS_OAI222X2 U510 ( .A(n281), .B(n448), .C(n375), .D(n450), .E(n288), 
        .F(n451), .Z(n679) );
  HS65_GS_IVX9 U511 ( .A(OBSERVE_r5[4]), .Z(n375) );
  HS65_GS_OAI222X2 U512 ( .A(n281), .B(n432), .C(n374), .D(n434), .E(n288), 
        .F(n435), .Z(n663) );
  HS65_GS_IVX9 U513 ( .A(OBSERVE_r6[4]), .Z(n374) );
  HS65_GS_OAI222X2 U514 ( .A(n281), .B(n406), .C(n369), .D(n408), .E(n288), 
        .F(n409), .Z(n615) );
  HS65_GS_IVX9 U515 ( .A(OBSERVE_r9[4]), .Z(n369) );
  HS65_GS_OAI222X2 U516 ( .A(n282), .B(n432), .C(n359), .D(n434), .E(n289), 
        .F(n435), .Z(n664) );
  HS65_GS_IVX9 U517 ( .A(OBSERVE_r6[3]), .Z(n359) );
  HS65_GS_OAI222X2 U518 ( .A(n282), .B(n427), .C(n358), .D(n428), .E(n289), 
        .F(n429), .Z(n648) );
  HS65_GS_IVX9 U519 ( .A(OBSERVE_r7[3]), .Z(n358) );
  HS65_GS_AOI212X4 U520 ( .A(OBSERVE_r9[1]), .B(n772), .C(n773), .D(
        OBSERVE_r8[1]), .E(n94), .Z(n93) );
  HS65_GS_OAI22X6 U521 ( .A(n31), .B(n331), .C(n10), .D(n329), .Z(n94) );
  HS65_GS_AOI212X4 U522 ( .A(OBSERVE_r9[2]), .B(n772), .C(n773), .D(
        OBSERVE_r8[2]), .E(n84), .Z(n83) );
  HS65_GS_OAI22X6 U523 ( .A(n31), .B(n344), .C(n10), .D(n343), .Z(n84) );
  HS65_GS_AOI212X4 U524 ( .A(OBSERVE_r9[8]), .B(n772), .C(n773), .D(
        OBSERVE_r8[8]), .E(n30), .Z(n28) );
  HS65_GS_OAI22X6 U525 ( .A(n31), .B(n436), .C(n10), .D(n433), .Z(n30) );
  HS65_GS_AOI212X4 U526 ( .A(OBSERVE_r15[3]), .B(n252), .C(OBSERVE_r1[3]), .D(
        n778), .E(n77), .Z(n70) );
  HS65_GS_OAI22X6 U527 ( .A(n35), .B(n361), .C(n37), .D(n360), .Z(n77) );
  HS65_GS_AOI212X4 U528 ( .A(inst_dest[5]), .B(OBSERVE_r5[14]), .C(
        inst_dest[6]), .D(OBSERVE_r6[14]), .E(n223), .Z(n222) );
  HS65_GS_OAI22X6 U529 ( .A(n758), .B(n784), .C(n295), .D(n783), .Z(n223) );
  HS65_GS_AOI212X4 U530 ( .A(inst_dest[5]), .B(OBSERVE_r5[9]), .C(inst_dest[6]), .D(OBSERVE_r6[9]), .E(n152), .Z(n151) );
  HS65_GS_OAI22X6 U531 ( .A(n453), .B(n784), .C(n300), .D(n783), .Z(n152) );
  HS65_GS_AOI212X4 U532 ( .A(inst_dest[5]), .B(OBSERVE_r5[13]), .C(
        inst_dest[6]), .D(OBSERVE_r6[13]), .E(n230), .Z(n229) );
  HS65_GS_OAI22X6 U533 ( .A(n746), .B(n784), .C(n296), .D(n783), .Z(n230) );
  HS65_GS_AOI212X4 U534 ( .A(inst_dest[5]), .B(OBSERVE_r5[12]), .C(
        inst_dest[6]), .D(OBSERVE_r6[12]), .E(n237), .Z(n236) );
  HS65_GS_OAI22X6 U535 ( .A(n734), .B(n784), .C(n297), .D(n783), .Z(n237) );
  HS65_GS_AOI212X4 U536 ( .A(inst_dest[5]), .B(OBSERVE_r5[11]), .C(
        inst_dest[6]), .D(OBSERVE_r6[11]), .E(n244), .Z(n243) );
  HS65_GS_OAI22X6 U537 ( .A(n722), .B(n784), .C(n298), .D(n783), .Z(n244) );
  HS65_GS_AOI212X4 U538 ( .A(inst_dest[5]), .B(OBSERVE_r5[10]), .C(
        inst_dest[6]), .D(OBSERVE_r6[10]), .E(n251), .Z(n250) );
  HS65_GS_OAI22X6 U539 ( .A(n471), .B(n784), .C(n299), .D(n783), .Z(n251) );
  HS65_GS_AOI212X4 U540 ( .A(inst_dest[5]), .B(OBSERVE_r5[8]), .C(inst_dest[6]), .D(OBSERVE_r6[8]), .E(n161), .Z(n160) );
  HS65_GS_OAI22X6 U541 ( .A(n438), .B(n784), .C(n301), .D(n783), .Z(n161) );
  HS65_GS_AOI212X4 U542 ( .A(inst_dest[5]), .B(OBSERVE_r5[6]), .C(inst_dest[6]), .D(OBSERVE_r6[6]), .E(n175), .Z(n174) );
  HS65_GS_OAI22X6 U543 ( .A(n403), .B(n784), .C(n303), .D(n783), .Z(n175) );
  HS65_GS_AOI212X4 U544 ( .A(inst_dest[5]), .B(OBSERVE_r5[2]), .C(inst_dest[6]), .D(OBSERVE_r6[2]), .E(n202), .Z(n201) );
  HS65_GS_OAI22X6 U545 ( .A(n346), .B(n784), .C(n306), .D(n783), .Z(n202) );
  HS65_GS_AOI212X4 U546 ( .A(inst_dest[5]), .B(OBSERVE_r5[1]), .C(inst_dest[6]), .D(OBSERVE_r6[1]), .E(n209), .Z(n208) );
  HS65_GS_OAI22X6 U547 ( .A(n335), .B(n784), .C(n307), .D(n783), .Z(n209) );
  HS65_GS_AOI212X4 U548 ( .A(inst_dest[5]), .B(OBSERVE_r5[4]), .C(inst_dest[6]), .D(OBSERVE_r6[4]), .E(n189), .Z(n188) );
  HS65_GS_OAI22X6 U549 ( .A(n376), .B(n784), .C(n305), .D(n783), .Z(n189) );
  HS65_GS_AOI212X4 U550 ( .A(inst_dest[5]), .B(OBSERVE_r5[5]), .C(inst_dest[6]), .D(OBSERVE_r6[5]), .E(n182), .Z(n181) );
  HS65_GS_OAI22X6 U551 ( .A(n391), .B(n784), .C(n304), .D(n783), .Z(n182) );
  HS65_GS_AOI212X4 U552 ( .A(inst_dest[5]), .B(OBSERVE_r5[15]), .C(
        inst_dest[6]), .D(OBSERVE_r6[15]), .E(n216), .Z(n215) );
  HS65_GS_OAI22X6 U553 ( .A(n770), .B(n784), .C(n294), .D(n783), .Z(n216) );
  HS65_GS_AOI212X4 U554 ( .A(inst_dest[5]), .B(OBSERVE_r5[7]), .C(inst_dest[6]), .D(OBSERVE_r6[7]), .E(n168), .Z(n167) );
  HS65_GS_OAI22X6 U555 ( .A(n418), .B(n784), .C(n302), .D(n783), .Z(n168) );
  HS65_GS_NAND4ABX3 U556 ( .A(n219), .B(n220), .C(n221), .D(n222), .Z(
        reg_dest[14]) );
  HS65_GS_AOI222X2 U557 ( .A(inst_dest[9]), .B(OBSERVE_r9[14]), .C(
        inst_dest[7]), .D(OBSERVE_r7[14]), .E(inst_dest[8]), .F(OBSERVE_r8[14]), .Z(n221) );
  HS65_GS_MX41X7 U558 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[14]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[14]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[14]), .D3(inst_dest[13]), .S3(OBSERVE_r13[14]), .Z(n220)
         );
  HS65_GS_MX41X7 U559 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[14]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[14]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[14]), .D3(inst_dest[0]), .S3(pc[14]), .Z(n219) );
  HS65_GS_NAND4ABX3 U560 ( .A(n148), .B(n149), .C(n150), .D(n151), .Z(
        reg_dest[9]) );
  HS65_GS_AOI222X2 U561 ( .A(inst_dest[9]), .B(OBSERVE_r9[9]), .C(inst_dest[7]), .D(OBSERVE_r7[9]), .E(inst_dest[8]), .F(OBSERVE_r8[9]), .Z(n150) );
  HS65_GS_MX41X7 U562 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[9]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[9]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[9]), .D3(inst_dest[13]), .S3(OBSERVE_r13[9]), .Z(n149) );
  HS65_GS_MX41X7 U563 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[9]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[9]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[9]), .D3(inst_dest[0]), .S3(pc[9]), .Z(n148) );
  HS65_GS_NAND4ABX3 U564 ( .A(n226), .B(n227), .C(n228), .D(n229), .Z(
        reg_dest[13]) );
  HS65_GS_AOI222X2 U565 ( .A(inst_dest[9]), .B(OBSERVE_r9[13]), .C(
        inst_dest[7]), .D(OBSERVE_r7[13]), .E(inst_dest[8]), .F(OBSERVE_r8[13]), .Z(n228) );
  HS65_GS_MX41X7 U566 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[13]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[13]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[13]), .D3(inst_dest[13]), .S3(OBSERVE_r13[13]), .Z(n227)
         );
  HS65_GS_MX41X7 U567 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[13]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[13]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[13]), .D3(inst_dest[0]), .S3(pc[13]), .Z(n226) );
  HS65_GS_NAND4ABX3 U568 ( .A(n233), .B(n234), .C(n235), .D(n236), .Z(
        reg_dest[12]) );
  HS65_GS_AOI222X2 U569 ( .A(inst_dest[9]), .B(OBSERVE_r9[12]), .C(
        inst_dest[7]), .D(OBSERVE_r7[12]), .E(inst_dest[8]), .F(OBSERVE_r8[12]), .Z(n235) );
  HS65_GS_MX41X7 U570 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[12]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[12]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[12]), .D3(inst_dest[13]), .S3(OBSERVE_r13[12]), .Z(n234)
         );
  HS65_GS_MX41X7 U571 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[12]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[12]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[12]), .D3(inst_dest[0]), .S3(pc[12]), .Z(n233) );
  HS65_GS_NAND4ABX3 U572 ( .A(n240), .B(n241), .C(n242), .D(n243), .Z(
        reg_dest[11]) );
  HS65_GS_AOI222X2 U573 ( .A(inst_dest[9]), .B(OBSERVE_r9[11]), .C(
        inst_dest[7]), .D(OBSERVE_r7[11]), .E(inst_dest[8]), .F(OBSERVE_r8[11]), .Z(n242) );
  HS65_GS_MX41X7 U574 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[11]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[11]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[11]), .D3(inst_dest[13]), .S3(OBSERVE_r13[11]), .Z(n241)
         );
  HS65_GS_MX41X7 U575 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[11]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[11]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[11]), .D3(inst_dest[0]), .S3(pc[11]), .Z(n240) );
  HS65_GS_NAND4ABX3 U576 ( .A(n247), .B(n248), .C(n249), .D(n250), .Z(
        reg_dest[10]) );
  HS65_GS_AOI222X2 U577 ( .A(inst_dest[9]), .B(OBSERVE_r9[10]), .C(
        inst_dest[7]), .D(OBSERVE_r7[10]), .E(inst_dest[8]), .F(OBSERVE_r8[10]), .Z(n249) );
  HS65_GS_MX41X7 U578 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[10]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[10]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[10]), .D3(inst_dest[13]), .S3(OBSERVE_r13[10]), .Z(n248)
         );
  HS65_GS_MX41X7 U579 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[10]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[10]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[10]), .D3(inst_dest[0]), .S3(pc[10]), .Z(n247) );
  HS65_GS_NAND4ABX3 U580 ( .A(n171), .B(n172), .C(n173), .D(n174), .Z(
        reg_dest[6]) );
  HS65_GS_AOI222X2 U581 ( .A(inst_dest[9]), .B(OBSERVE_r9[6]), .C(inst_dest[7]), .D(OBSERVE_r7[6]), .E(inst_dest[8]), .F(OBSERVE_r8[6]), .Z(n173) );
  HS65_GS_MX41X7 U582 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[6]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[6]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[6]), .D3(inst_dest[13]), .S3(OBSERVE_r13[6]), .Z(n172) );
  HS65_GS_MX41X7 U583 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[6]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[6]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[6]), .D3(inst_dest[0]), .S3(pc[6]), .Z(n171) );
  HS65_GS_NAND4ABX3 U584 ( .A(n185), .B(n186), .C(n187), .D(n188), .Z(
        reg_dest[4]) );
  HS65_GS_AOI222X2 U585 ( .A(inst_dest[9]), .B(OBSERVE_r9[4]), .C(inst_dest[7]), .D(OBSERVE_r7[4]), .E(inst_dest[8]), .F(OBSERVE_r8[4]), .Z(n187) );
  HS65_GS_MX41X7 U586 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[4]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[4]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[4]), .D3(inst_dest[13]), .S3(OBSERVE_r13[4]), .Z(n186) );
  HS65_GS_MX41X7 U587 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[4]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[4]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[4]), .D3(inst_dest[0]), .S3(pc[4]), .Z(n185) );
  HS65_GS_NAND4ABX3 U588 ( .A(n178), .B(n179), .C(n180), .D(n181), .Z(
        reg_dest[5]) );
  HS65_GS_AOI222X2 U589 ( .A(inst_dest[9]), .B(OBSERVE_r9[5]), .C(inst_dest[7]), .D(OBSERVE_r7[5]), .E(inst_dest[8]), .F(OBSERVE_r8[5]), .Z(n180) );
  HS65_GS_MX41X7 U590 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[5]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[5]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[5]), .D3(inst_dest[13]), .S3(OBSERVE_r13[5]), .Z(n179) );
  HS65_GS_MX41X7 U591 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[5]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[5]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[5]), .D3(inst_dest[0]), .S3(pc[5]), .Z(n178) );
  HS65_GS_NAND4ABX3 U592 ( .A(n212), .B(n213), .C(n214), .D(n215), .Z(
        reg_dest[15]) );
  HS65_GS_AOI222X2 U593 ( .A(inst_dest[9]), .B(OBSERVE_r9[15]), .C(
        inst_dest[7]), .D(OBSERVE_r7[15]), .E(inst_dest[8]), .F(OBSERVE_r8[15]), .Z(n214) );
  HS65_GS_MX41X7 U594 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[15]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[15]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[15]), .D3(inst_dest[13]), .S3(OBSERVE_r13[15]), .Z(n213)
         );
  HS65_GS_MX41X7 U595 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[15]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[15]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[15]), .D3(inst_dest[0]), .S3(pc[15]), .Z(n212) );
  HS65_GS_NAND4ABX3 U596 ( .A(n164), .B(n165), .C(n166), .D(n167), .Z(
        reg_dest[7]) );
  HS65_GS_AOI222X2 U597 ( .A(inst_dest[9]), .B(OBSERVE_r9[7]), .C(inst_dest[7]), .D(OBSERVE_r7[7]), .E(inst_dest[8]), .F(OBSERVE_r8[7]), .Z(n166) );
  HS65_GS_MX41X7 U598 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[7]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[7]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[7]), .D3(inst_dest[13]), .S3(OBSERVE_r13[7]), .Z(n165) );
  HS65_GS_MX41X7 U599 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[7]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[7]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[7]), .D3(inst_dest[0]), .S3(pc[7]), .Z(n164) );
  HS65_GS_NAND4ABX3 U600 ( .A(n192), .B(n193), .C(n194), .D(n195), .Z(
        reg_dest[3]) );
  HS65_GS_AOI212X4 U601 ( .A(inst_dest[4]), .B(OBSERVE_r4[3]), .C(inst_dest[5]), .D(OBSERVE_r5[3]), .E(n196), .Z(n195) );
  HS65_GS_MX41X7 U602 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[3]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[3]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[3]), .D3(inst_dest[13]), .S3(OBSERVE_r13[3]), .Z(n193) );
  HS65_GS_AOI212X4 U603 ( .A(inst_dest[8]), .B(OBSERVE_r8[3]), .C(inst_dest[9]), .D(OBSERVE_r9[3]), .E(n197), .Z(n194) );
  HS65_GS_NAND4ABX3 U604 ( .A(n157), .B(n158), .C(n159), .D(n160), .Z(
        reg_dest[8]) );
  HS65_GS_MX41X7 U605 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[8]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[8]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[8]), .D3(inst_dest[13]), .S3(OBSERVE_r13[8]), .Z(n158) );
  HS65_GS_AOI212X4 U606 ( .A(inst_dest[7]), .B(OBSERVE_r7[8]), .C(inst_dest[9]), .D(OBSERVE_r9[8]), .E(n163), .Z(n159) );
  HS65_GS_MX41X7 U607 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[8]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[8]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[8]), .D3(inst_dest[0]), .S3(pc[8]), .Z(n157) );
  HS65_GS_NAND4ABX3 U608 ( .A(n198), .B(n199), .C(n200), .D(n201), .Z(
        reg_dest[2]) );
  HS65_GS_MX41X7 U609 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[2]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[2]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[2]), .D3(inst_dest[13]), .S3(OBSERVE_r13[2]), .Z(n199) );
  HS65_GS_AOI212X4 U610 ( .A(inst_dest[7]), .B(OBSERVE_r7[2]), .C(inst_dest[9]), .D(OBSERVE_r9[2]), .E(n204), .Z(n200) );
  HS65_GS_MX41X7 U611 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[2]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[2]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[2]), .D3(inst_dest[0]), .S3(pc[2]), .Z(n198) );
  HS65_GS_NAND4ABX3 U612 ( .A(n205), .B(n206), .C(n207), .D(n208), .Z(
        reg_dest[1]) );
  HS65_GS_MX41X7 U613 ( .D0(inst_dest[1]), .S0(OBSERVE_r1[1]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[1]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[1]), .D3(inst_dest[13]), .S3(OBSERVE_r13[1]), .Z(n206) );
  HS65_GS_AOI212X4 U614 ( .A(inst_dest[7]), .B(OBSERVE_r7[1]), .C(inst_dest[9]), .D(OBSERVE_r9[1]), .E(n211), .Z(n207) );
  HS65_GS_MX41X7 U615 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[1]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[1]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[1]), .D3(inst_dest[0]), .S3(pc[1]), .Z(n205) );
  HS65_GS_NAND4ABX3 U616 ( .A(n90), .B(n91), .C(n92), .D(n93), .Z(reg_src[1])
         );
  HS65_GS_MX41X7 U617 ( .D0(OBSERVE_r10[1]), .S0(n18), .D1(OBSERVE_r11[1]), 
        .S1(n19), .D2(OBSERVE_r12[1]), .S2(n20), .D3(OBSERVE_r13[1]), .S3(n21), 
        .Z(n91) );
  HS65_GS_MX41X7 U618 ( .D0(OBSERVE_r14[1]), .S0(n22), .D1(OBSERVE_r3[1]), 
        .S1(n23), .D2(status[1]), .S2(n39), .D3(pc[1]), .S3(n24), .Z(n90) );
  HS65_GS_AOI212X4 U619 ( .A(OBSERVE_r15[1]), .B(n252), .C(OBSERVE_r1[1]), .D(
        n778), .E(n97), .Z(n92) );
  HS65_GS_NAND4ABX3 U620 ( .A(n142), .B(n143), .C(n144), .D(n145), .Z(
        reg_src[0]) );
  HS65_GS_MX41X7 U621 ( .D0(OBSERVE_r10[0]), .S0(n18), .D1(OBSERVE_r11[0]), 
        .S1(n19), .D2(OBSERVE_r12[0]), .S2(n20), .D3(OBSERVE_r13[0]), .S3(n21), 
        .Z(n143) );
  HS65_GS_AOI212X4 U622 ( .A(OBSERVE_r3[0]), .B(n23), .C(OBSERVE_r14[0]), .D(
        n22), .E(n146), .Z(n145) );
  HS65_GS_AOI222X2 U623 ( .A(OBSERVE_r15[0]), .B(n252), .C(OBSERVE_r5[0]), .D(
        n776), .E(OBSERVE_r4[0]), .F(n777), .Z(n144) );
  HS65_GS_NAND4ABX3 U624 ( .A(n61), .B(n62), .C(n63), .D(n64), .Z(reg_src[4])
         );
  HS65_GS_MX41X7 U625 ( .D0(OBSERVE_r13[4]), .S0(n21), .D1(OBSERVE_r14[4]), 
        .S1(n22), .D2(OBSERVE_r3[4]), .S2(n23), .D3(pc[4]), .S3(n24), .Z(n61)
         );
  HS65_GS_AOI212X4 U626 ( .A(OBSERVE_r6[4]), .B(n775), .C(OBSERVE_r5[4]), .D(
        n776), .E(n65), .Z(n64) );
  HS65_GS_MX41X7 U627 ( .D0(OBSERVE_r9[4]), .S0(n772), .D1(OBSERVE_r10[4]), 
        .S1(n18), .D2(OBSERVE_r11[4]), .S2(n19), .D3(OBSERVE_r12[4]), .S3(n20), 
        .Z(n62) );
  HS65_GS_NAND4ABX3 U628 ( .A(n54), .B(n55), .C(n56), .D(n57), .Z(reg_src[5])
         );
  HS65_GS_MX41X7 U629 ( .D0(OBSERVE_r13[5]), .S0(n21), .D1(OBSERVE_r14[5]), 
        .S1(n22), .D2(OBSERVE_r3[5]), .S2(n23), .D3(pc[5]), .S3(n24), .Z(n54)
         );
  HS65_GS_AOI212X4 U630 ( .A(OBSERVE_r6[5]), .B(n775), .C(OBSERVE_r5[5]), .D(
        n776), .E(n58), .Z(n57) );
  HS65_GS_MX41X7 U631 ( .D0(OBSERVE_r9[5]), .S0(n772), .D1(OBSERVE_r10[5]), 
        .S1(n18), .D2(OBSERVE_r11[5]), .S2(n19), .D3(OBSERVE_r12[5]), .S3(n20), 
        .Z(n55) );
  HS65_GS_NAND4ABX3 U632 ( .A(n47), .B(n48), .C(n49), .D(n50), .Z(reg_src[6])
         );
  HS65_GS_MX41X7 U633 ( .D0(OBSERVE_r13[6]), .S0(n21), .D1(OBSERVE_r14[6]), 
        .S1(n22), .D2(OBSERVE_r3[6]), .S2(n23), .D3(pc[6]), .S3(n24), .Z(n47)
         );
  HS65_GS_AOI212X4 U634 ( .A(OBSERVE_r6[6]), .B(n775), .C(OBSERVE_r5[6]), .D(
        n776), .E(n51), .Z(n50) );
  HS65_GS_MX41X7 U635 ( .D0(OBSERVE_r9[6]), .S0(n772), .D1(OBSERVE_r10[6]), 
        .S1(n18), .D2(OBSERVE_r11[6]), .S2(n19), .D3(OBSERVE_r12[6]), .S3(n20), 
        .Z(n48) );
  HS65_GS_NAND4ABX3 U636 ( .A(n40), .B(n41), .C(n42), .D(n43), .Z(reg_src[7])
         );
  HS65_GS_MX41X7 U637 ( .D0(OBSERVE_r13[7]), .S0(n21), .D1(OBSERVE_r14[7]), 
        .S1(n22), .D2(OBSERVE_r3[7]), .S2(n23), .D3(pc[7]), .S3(n24), .Z(n40)
         );
  HS65_GS_AOI212X4 U638 ( .A(OBSERVE_r6[7]), .B(n775), .C(OBSERVE_r5[7]), .D(
        n776), .E(n44), .Z(n43) );
  HS65_GS_MX41X7 U639 ( .D0(OBSERVE_r9[7]), .S0(n772), .D1(OBSERVE_r10[7]), 
        .S1(n18), .D2(OBSERVE_r11[7]), .S2(n19), .D3(OBSERVE_r12[7]), .S3(n20), 
        .Z(n41) );
  HS65_GS_NAND4ABX3 U640 ( .A(n3), .B(n4), .C(n5), .D(n6), .Z(reg_src[9]) );
  HS65_GS_MX41X7 U641 ( .D0(OBSERVE_r13[9]), .S0(n21), .D1(OBSERVE_r14[9]), 
        .S1(n22), .D2(OBSERVE_r3[9]), .S2(n23), .D3(pc[9]), .S3(n24), .Z(n3)
         );
  HS65_GS_AOI212X4 U642 ( .A(OBSERVE_r6[9]), .B(n775), .C(OBSERVE_r5[9]), .D(
        n776), .E(n9), .Z(n6) );
  HS65_GS_MX41X7 U643 ( .D0(OBSERVE_r9[9]), .S0(n772), .D1(OBSERVE_r10[9]), 
        .S1(n18), .D2(OBSERVE_r11[9]), .S2(n19), .D3(OBSERVE_r12[9]), .S3(n20), 
        .Z(n4) );
  HS65_GS_NAND4ABX3 U644 ( .A(n135), .B(n136), .C(n137), .D(n138), .Z(
        reg_src[10]) );
  HS65_GS_MX41X7 U645 ( .D0(OBSERVE_r13[10]), .S0(n21), .D1(OBSERVE_r14[10]), 
        .S1(n22), .D2(OBSERVE_r3[10]), .S2(n23), .D3(pc[10]), .S3(n24), .Z(
        n135) );
  HS65_GS_AOI212X4 U646 ( .A(OBSERVE_r6[10]), .B(n775), .C(OBSERVE_r5[10]), 
        .D(n776), .E(n139), .Z(n138) );
  HS65_GS_MX41X7 U647 ( .D0(OBSERVE_r9[10]), .S0(n772), .D1(OBSERVE_r10[10]), 
        .S1(n18), .D2(OBSERVE_r11[10]), .S2(n19), .D3(OBSERVE_r12[10]), .S3(
        n20), .Z(n136) );
  HS65_GS_NAND4ABX3 U648 ( .A(n128), .B(n129), .C(n130), .D(n131), .Z(
        reg_src[11]) );
  HS65_GS_MX41X7 U649 ( .D0(OBSERVE_r13[11]), .S0(n21), .D1(OBSERVE_r14[11]), 
        .S1(n22), .D2(OBSERVE_r3[11]), .S2(n23), .D3(pc[11]), .S3(n24), .Z(
        n128) );
  HS65_GS_AOI212X4 U650 ( .A(OBSERVE_r6[11]), .B(n775), .C(OBSERVE_r5[11]), 
        .D(n776), .E(n132), .Z(n131) );
  HS65_GS_MX41X7 U651 ( .D0(OBSERVE_r9[11]), .S0(n772), .D1(OBSERVE_r10[11]), 
        .S1(n18), .D2(OBSERVE_r11[11]), .S2(n19), .D3(OBSERVE_r12[11]), .S3(
        n20), .Z(n129) );
  HS65_GS_NAND4ABX3 U652 ( .A(n121), .B(n122), .C(n123), .D(n124), .Z(
        reg_src[12]) );
  HS65_GS_MX41X7 U653 ( .D0(OBSERVE_r13[12]), .S0(n21), .D1(OBSERVE_r14[12]), 
        .S1(n22), .D2(OBSERVE_r3[12]), .S2(n23), .D3(pc[12]), .S3(n24), .Z(
        n121) );
  HS65_GS_AOI212X4 U654 ( .A(OBSERVE_r6[12]), .B(n775), .C(OBSERVE_r5[12]), 
        .D(n776), .E(n125), .Z(n124) );
  HS65_GS_MX41X7 U655 ( .D0(OBSERVE_r9[12]), .S0(n772), .D1(OBSERVE_r10[12]), 
        .S1(n18), .D2(OBSERVE_r11[12]), .S2(n19), .D3(OBSERVE_r12[12]), .S3(
        n20), .Z(n122) );
  HS65_GS_NAND4ABX3 U656 ( .A(n114), .B(n115), .C(n116), .D(n117), .Z(
        reg_src[13]) );
  HS65_GS_MX41X7 U657 ( .D0(OBSERVE_r13[13]), .S0(n21), .D1(OBSERVE_r14[13]), 
        .S1(n22), .D2(OBSERVE_r3[13]), .S2(n23), .D3(pc[13]), .S3(n24), .Z(
        n114) );
  HS65_GS_AOI212X4 U658 ( .A(OBSERVE_r6[13]), .B(n775), .C(OBSERVE_r5[13]), 
        .D(n776), .E(n118), .Z(n117) );
  HS65_GS_MX41X7 U659 ( .D0(OBSERVE_r9[13]), .S0(n772), .D1(OBSERVE_r10[13]), 
        .S1(n18), .D2(OBSERVE_r11[13]), .S2(n19), .D3(OBSERVE_r12[13]), .S3(
        n20), .Z(n115) );
  HS65_GS_NAND4ABX3 U660 ( .A(n107), .B(n108), .C(n109), .D(n110), .Z(
        reg_src[14]) );
  HS65_GS_MX41X7 U661 ( .D0(OBSERVE_r13[14]), .S0(n21), .D1(OBSERVE_r14[14]), 
        .S1(n22), .D2(OBSERVE_r3[14]), .S2(n23), .D3(pc[14]), .S3(n24), .Z(
        n107) );
  HS65_GS_AOI212X4 U662 ( .A(OBSERVE_r6[14]), .B(n775), .C(OBSERVE_r5[14]), 
        .D(n776), .E(n111), .Z(n110) );
  HS65_GS_MX41X7 U663 ( .D0(OBSERVE_r9[14]), .S0(n772), .D1(OBSERVE_r10[14]), 
        .S1(n18), .D2(OBSERVE_r11[14]), .S2(n19), .D3(OBSERVE_r12[14]), .S3(
        n20), .Z(n108) );
  HS65_GS_AND2X4 U664 ( .A(inst_src[0]), .B(n779), .Z(n24) );
  HS65_GS_NAND4ABX3 U665 ( .A(n68), .B(n69), .C(n70), .D(n71), .Z(reg_src[3])
         );
  HS65_GS_MX41X7 U666 ( .D0(OBSERVE_r10[3]), .S0(n18), .D1(OBSERVE_r11[3]), 
        .S1(n19), .D2(OBSERVE_r12[3]), .S2(n20), .D3(OBSERVE_r13[3]), .S3(n21), 
        .Z(n69) );
  HS65_GS_MX41X7 U667 ( .D0(OBSERVE_r14[3]), .S0(n22), .D1(OBSERVE_r3[3]), 
        .S1(n23), .D2(gie), .S2(n39), .D3(pc[3]), .S3(n24), .Z(n68) );
  HS65_GS_AOI212X4 U668 ( .A(OBSERVE_r7[3]), .B(n774), .C(OBSERVE_r6[3]), .D(
        n775), .E(n73), .Z(n71) );
  HS65_GS_NAND4ABX3 U669 ( .A(n80), .B(n81), .C(n82), .D(n83), .Z(reg_src[2])
         );
  HS65_GS_MX41X7 U670 ( .D0(OBSERVE_r10[2]), .S0(n18), .D1(OBSERVE_r11[2]), 
        .S1(n19), .D2(OBSERVE_r12[2]), .S2(n20), .D3(OBSERVE_r13[2]), .S3(n21), 
        .Z(n81) );
  HS65_GS_MX41X7 U671 ( .D0(OBSERVE_r14[2]), .S0(n22), .D1(OBSERVE_r3[2]), 
        .S1(n23), .D2(status[2]), .S2(n39), .D3(pc[2]), .S3(n24), .Z(n80) );
  HS65_GS_AOI212X4 U672 ( .A(OBSERVE_r15[2]), .B(n252), .C(OBSERVE_r1[2]), .D(
        n778), .E(n87), .Z(n82) );
  HS65_GS_NAND4ABX3 U673 ( .A(n25), .B(n26), .C(n27), .D(n28), .Z(reg_src[8])
         );
  HS65_GS_MX41X7 U674 ( .D0(OBSERVE_r10[8]), .S0(n18), .D1(OBSERVE_r11[8]), 
        .S1(n19), .D2(OBSERVE_r12[8]), .S2(n20), .D3(OBSERVE_r13[8]), .S3(n21), 
        .Z(n26) );
  HS65_GS_MX41X7 U675 ( .D0(OBSERVE_r14[8]), .S0(n22), .D1(OBSERVE_r3[8]), 
        .S1(n23), .D2(status[3]), .S2(n39), .D3(pc[8]), .S3(n24), .Z(n25) );
  HS65_GS_AOI212X4 U676 ( .A(OBSERVE_r15[8]), .B(n252), .C(OBSERVE_r1[8]), .D(
        n778), .E(n34), .Z(n27) );
  HS65_GS_NAND4ABX3 U677 ( .A(n254), .B(n255), .C(n256), .D(n257), .Z(
        reg_dest[0]) );
  HS65_GS_AOI222X2 U678 ( .A(inst_dest[8]), .B(OBSERVE_r8[0]), .C(inst_dest[9]), .D(OBSERVE_r9[0]), .E(inst_dest[2]), .F(status[0]), .Z(n256) );
  HS65_GS_MX41X7 U679 ( .D0(inst_dest[3]), .S0(OBSERVE_r3[0]), .D1(
        inst_dest[15]), .S1(OBSERVE_r15[0]), .D2(inst_dest[14]), .S2(
        OBSERVE_r14[0]), .D3(inst_dest[13]), .S3(OBSERVE_r13[0]), .Z(n255) );
  HS65_GS_AOI212X4 U680 ( .A(inst_dest[6]), .B(OBSERVE_r6[0]), .C(inst_dest[7]), .D(OBSERVE_r7[0]), .E(n258), .Z(n257) );
  HS65_GS_NAND4ABX3 U681 ( .A(n100), .B(n101), .C(n102), .D(n103), .Z(
        reg_src[15]) );
  HS65_GS_MX41X7 U682 ( .D0(OBSERVE_r13[15]), .S0(n21), .D1(OBSERVE_r14[15]), 
        .S1(n22), .D2(OBSERVE_r3[15]), .S2(n23), .D3(pc[15]), .S3(n24), .Z(
        n100) );
  HS65_GS_AOI212X4 U683 ( .A(OBSERVE_r6[15]), .B(n775), .C(OBSERVE_r5[15]), 
        .D(n776), .E(n104), .Z(n103) );
  HS65_GS_MX41X7 U684 ( .D0(OBSERVE_r9[15]), .S0(n772), .D1(OBSERVE_r10[15]), 
        .S1(n18), .D2(OBSERVE_r11[15]), .S2(n19), .D3(OBSERVE_r12[15]), .S3(
        n20), .Z(n101) );
  HS65_GS_IVX9 U685 ( .A(OBSERVE_r4[1]), .Z(n335) );
  HS65_GS_IVX9 U686 ( .A(OBSERVE_r4[2]), .Z(n346) );
  HS65_GS_IVX9 U687 ( .A(OBSERVE_r4[8]), .Z(n438) );
  HS65_GS_CBI4I6X5 U688 ( .A(n293), .B(n782), .C(n480), .D(reg_sr_clr), .Z(N79) );
  HS65_GS_IVX9 U689 ( .A(gie), .Z(n293) );
  HS65_GS_NAND2X7 U690 ( .A(reg_dest_val[3]), .B(n782), .Z(n480) );
  HS65_GS_IVX9 U691 ( .A(OBSERVE_r4[6]), .Z(n403) );
  HS65_GS_IVX9 U692 ( .A(OBSERVE_r4[4]), .Z(n376) );
  HS65_GS_IVX9 U693 ( .A(OBSERVE_r4[5]), .Z(n391) );
  HS65_GS_IVX9 U694 ( .A(OBSERVE_r4[7]), .Z(n418) );
  HS65_GS_IVX9 U695 ( .A(OBSERVE_r4[14]), .Z(n758) );
  HS65_GS_IVX9 U696 ( .A(OBSERVE_r4[9]), .Z(n453) );
  HS65_GS_IVX9 U697 ( .A(OBSERVE_r4[13]), .Z(n746) );
  HS65_GS_IVX9 U698 ( .A(OBSERVE_r4[12]), .Z(n734) );
  HS65_GS_IVX9 U699 ( .A(OBSERVE_r4[11]), .Z(n722) );
  HS65_GS_IVX9 U700 ( .A(OBSERVE_r4[10]), .Z(n471) );
  HS65_GS_IVX9 U701 ( .A(OBSERVE_r8[4]), .Z(n372) );
  HS65_GS_IVX9 U702 ( .A(OBSERVE_r8[5]), .Z(n384) );
  HS65_GS_IVX9 U703 ( .A(OBSERVE_r8[6]), .Z(n399) );
  HS65_GS_IVX9 U704 ( .A(OBSERVE_r8[7]), .Z(n414) );
  HS65_GS_IVX9 U705 ( .A(OBSERVE_r8[9]), .Z(n446) );
  HS65_GS_IVX9 U706 ( .A(OBSERVE_r8[10]), .Z(n461) );
  HS65_GS_IVX9 U707 ( .A(OBSERVE_r8[11]), .Z(n718) );
  HS65_GS_IVX9 U708 ( .A(OBSERVE_r8[12]), .Z(n730) );
  HS65_GS_IVX9 U709 ( .A(OBSERVE_r8[13]), .Z(n742) );
  HS65_GS_IVX9 U710 ( .A(OBSERVE_r8[14]), .Z(n754) );
  HS65_GS_IVX9 U711 ( .A(OBSERVE_r7[4]), .Z(n373) );
  HS65_GS_IVX9 U712 ( .A(OBSERVE_r7[5]), .Z(n385) );
  HS65_GS_IVX9 U713 ( .A(OBSERVE_r7[6]), .Z(n400) );
  HS65_GS_IVX9 U714 ( .A(OBSERVE_r7[7]), .Z(n415) );
  HS65_GS_IVX9 U715 ( .A(OBSERVE_r7[9]), .Z(n447) );
  HS65_GS_IVX9 U716 ( .A(OBSERVE_r7[10]), .Z(n462) );
  HS65_GS_IVX9 U717 ( .A(OBSERVE_r7[11]), .Z(n719) );
  HS65_GS_IVX9 U718 ( .A(OBSERVE_r7[12]), .Z(n731) );
  HS65_GS_IVX9 U719 ( .A(OBSERVE_r7[13]), .Z(n743) );
  HS65_GS_IVX9 U720 ( .A(OBSERVE_r7[14]), .Z(n755) );
  HS65_GS_IVX9 U721 ( .A(OBSERVE_r5[3]), .Z(n360) );
  HS65_GS_IVX9 U722 ( .A(OBSERVE_r9[3]), .Z(n356) );
  HS65_GS_IVX9 U723 ( .A(OBSERVE_r7[1]), .Z(n329) );
  HS65_GS_IVX9 U724 ( .A(OBSERVE_r5[1]), .Z(n334) );
  HS65_GS_IVX9 U725 ( .A(OBSERVE_r7[2]), .Z(n343) );
  HS65_GS_IVX9 U726 ( .A(OBSERVE_r5[2]), .Z(n345) );
  HS65_GS_IVX9 U727 ( .A(OBSERVE_r7[8]), .Z(n433) );
  HS65_GS_IVX9 U728 ( .A(OBSERVE_r5[8]), .Z(n437) );
  HS65_GS_IVX9 U729 ( .A(OBSERVE_r6[1]), .Z(n331) );
  HS65_GS_IVX9 U730 ( .A(OBSERVE_r6[2]), .Z(n344) );
  HS65_GS_IVX9 U731 ( .A(OBSERVE_r6[8]), .Z(n436) );
  HS65_GS_IVX9 U732 ( .A(OBSERVE_r4[3]), .Z(n361) );
  HS65_GS_IVX9 U733 ( .A(OBSERVE_r8[3]), .Z(n357) );
  HS65_GS_MX41X7 U734 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[3]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[3]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[3]), .D3(inst_dest[0]), .S3(pc[3]), .Z(n192) );
  HS65_GS_MX41X7 U735 ( .D0(inst_dest[12]), .S0(OBSERVE_r12[0]), .D1(
        inst_dest[11]), .S1(OBSERVE_r11[0]), .D2(inst_dest[10]), .S2(
        OBSERVE_r10[0]), .D3(inst_dest[0]), .S3(pc[0]), .Z(n254) );
  HS65_GS_IVX9 U736 ( .A(OBSERVE_r3[8]), .Z(n301) );
  HS65_GS_IVX9 U737 ( .A(OBSERVE_r3[2]), .Z(n306) );
  HS65_GS_IVX9 U738 ( .A(OBSERVE_r3[1]), .Z(n307) );
  HS65_GS_IVX9 U739 ( .A(OBSERVE_r3[14]), .Z(n295) );
  HS65_GS_IVX9 U740 ( .A(OBSERVE_r3[9]), .Z(n300) );
  HS65_GS_IVX9 U741 ( .A(OBSERVE_r3[13]), .Z(n296) );
  HS65_GS_IVX9 U742 ( .A(OBSERVE_r3[12]), .Z(n297) );
  HS65_GS_IVX9 U743 ( .A(OBSERVE_r3[11]), .Z(n298) );
  HS65_GS_IVX9 U744 ( .A(OBSERVE_r3[10]), .Z(n299) );
  HS65_GS_IVX9 U745 ( .A(OBSERVE_r3[6]), .Z(n303) );
  HS65_GS_IVX9 U746 ( .A(OBSERVE_r3[4]), .Z(n305) );
  HS65_GS_IVX9 U747 ( .A(OBSERVE_r3[5]), .Z(n304) );
  HS65_GS_IVX9 U748 ( .A(OBSERVE_r3[7]), .Z(n302) );
  HS65_GS_AO22X9 U749 ( .A(n39), .B(status[0]), .C(n24), .D(pc[0]), .Z(n146)
         );
  HS65_GS_MX41X7 U750 ( .D0(OBSERVE_r6[0]), .S0(n775), .D1(OBSERVE_r7[0]), 
        .S1(n774), .D2(n773), .S2(OBSERVE_r8[0]), .D3(OBSERVE_r9[0]), .S3(n772), .Z(n142) );
  HS65_GS_AO22X9 U751 ( .A(OBSERVE_r8[2]), .B(inst_dest[8]), .C(status[2]), 
        .D(inst_dest[2]), .Z(n204) );
  HS65_GS_AO22X9 U752 ( .A(OBSERVE_r5[0]), .B(inst_dest[5]), .C(OBSERVE_r4[0]), 
        .D(inst_dest[4]), .Z(n258) );
  HS65_GS_AO22X9 U753 ( .A(OBSERVE_r8[1]), .B(inst_dest[8]), .C(status[1]), 
        .D(inst_dest[2]), .Z(n211) );
  HS65_GS_AO22X9 U754 ( .A(OBSERVE_r8[8]), .B(inst_dest[8]), .C(status[3]), 
        .D(inst_dest[2]), .Z(n163) );
  HS65_GS_AO22X9 U755 ( .A(OBSERVE_r3[3]), .B(inst_dest[3]), .C(gie), .D(
        inst_dest[2]), .Z(n196) );
  HS65_GS_MX41X7 U756 ( .D0(n781), .S0(reg_dest_val[7]), .D1(reg_sp_val[7]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[7]), .D3(n469), .S3(
        OBSERVE_r1[7]), .Z(n708) );
  HS65_GS_MX41X7 U757 ( .D0(n781), .S0(reg_dest_val[6]), .D1(reg_sp_val[6]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[6]), .D3(n469), .S3(
        OBSERVE_r1[6]), .Z(n709) );
  HS65_GS_MX41X7 U758 ( .D0(n781), .S0(reg_dest_val[5]), .D1(reg_sp_val[5]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[5]), .D3(n469), .S3(
        OBSERVE_r1[5]), .Z(n710) );
  HS65_GS_MX41X7 U759 ( .D0(n781), .S0(reg_dest_val[4]), .D1(reg_sp_val[4]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[4]), .D3(n469), .S3(
        OBSERVE_r1[4]), .Z(n711) );
  HS65_GS_MX41X7 U760 ( .D0(n781), .S0(reg_dest_val[3]), .D1(reg_sp_val[3]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[3]), .D3(n469), .S3(
        OBSERVE_r1[3]), .Z(n712) );
  HS65_GS_MX41X7 U761 ( .D0(reg_sp_val[12]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[12]), .D2(n469), .S2(OBSERVE_r1[12]), .D3(n781), .S3(
        pc_sw[12]), .Z(n703) );
  HS65_GS_MX41X7 U762 ( .D0(reg_sp_val[11]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[11]), .D2(n469), .S2(OBSERVE_r1[11]), .D3(n781), .S3(
        pc_sw[11]), .Z(n704) );
  HS65_GS_MX41X7 U763 ( .D0(reg_sp_val[10]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[10]), .D2(n469), .S2(OBSERVE_r1[10]), .D3(n781), .S3(
        pc_sw[10]), .Z(n705) );
  HS65_GS_MX41X7 U776 ( .D0(reg_sp_val[9]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[9]), .D2(n469), .S2(OBSERVE_r1[9]), .D3(n781), .S3(
        pc_sw[9]), .Z(n706) );
  HS65_GS_MX41X7 U777 ( .D0(reg_sp_val[8]), .S0(n467), .D1(n468), .S1(
        reg_incr_val[8]), .D2(n469), .S2(OBSERVE_r1[8]), .D3(n781), .S3(
        pc_sw[8]), .Z(n707) );
  HS65_GS_AO22X9 U778 ( .A(OBSERVE_r7[3]), .B(inst_dest[7]), .C(OBSERVE_r6[3]), 
        .D(inst_dest[6]), .Z(n197) );
  HS65_GS_AO22X9 U779 ( .A(n259), .B(OBSERVE_r3[3]), .C(n780), .D(
        reg_dest_val[3]), .Z(n494) );
  HS65_GS_OAI222X2 U780 ( .A(n283), .B(n309), .C(n337), .D(n311), .E(n290), 
        .F(n312), .Z(n537) );
  HS65_GS_IVX9 U781 ( .A(OBSERVE_r14[2]), .Z(n337) );
  HS65_GS_OAI222X2 U782 ( .A(n284), .B(n309), .C(n323), .D(n311), .E(n291), 
        .F(n312), .Z(n538) );
  HS65_GS_IVX9 U783 ( .A(OBSERVE_r14[1]), .Z(n323) );
  HS65_GS_OAI222X2 U784 ( .A(n283), .B(n387), .C(n341), .D(n389), .E(n290), 
        .F(n390), .Z(n601) );
  HS65_GS_IVX9 U785 ( .A(OBSERVE_r10[2]), .Z(n341) );
  HS65_GS_OAI222X2 U786 ( .A(n283), .B(n368), .C(n340), .D(n370), .E(n290), 
        .F(n371), .Z(n585) );
  HS65_GS_IVX9 U787 ( .A(OBSERVE_r11[2]), .Z(n340) );
  HS65_GS_OAI222X2 U788 ( .A(n283), .B(n349), .C(n339), .D(n351), .E(n290), 
        .F(n352), .Z(n569) );
  HS65_GS_IVX9 U789 ( .A(OBSERVE_r12[2]), .Z(n339) );
  HS65_GS_OAI222X2 U790 ( .A(n283), .B(n330), .C(n338), .D(n332), .E(n290), 
        .F(n333), .Z(n553) );
  HS65_GS_IVX9 U791 ( .A(OBSERVE_r13[2]), .Z(n338) );
  HS65_GS_OAI222X2 U792 ( .A(n284), .B(n387), .C(n327), .D(n389), .E(n291), 
        .F(n390), .Z(n602) );
  HS65_GS_IVX9 U793 ( .A(OBSERVE_r10[1]), .Z(n327) );
  HS65_GS_OAI222X2 U794 ( .A(n284), .B(n368), .C(n326), .D(n370), .E(n291), 
        .F(n371), .Z(n586) );
  HS65_GS_IVX9 U795 ( .A(OBSERVE_r11[1]), .Z(n326) );
  HS65_GS_OAI222X2 U796 ( .A(n284), .B(n349), .C(n325), .D(n351), .E(n291), 
        .F(n352), .Z(n570) );
  HS65_GS_IVX9 U797 ( .A(OBSERVE_r12[1]), .Z(n325) );
  HS65_GS_OAI222X2 U798 ( .A(n284), .B(n330), .C(n324), .D(n332), .E(n291), 
        .F(n333), .Z(n554) );
  HS65_GS_IVX9 U799 ( .A(OBSERVE_r13[1]), .Z(n324) );
  HS65_GS_OAI222X2 U800 ( .A(n133), .B(n387), .C(n316), .D(n389), .E(n292), 
        .F(n390), .Z(n603) );
  HS65_GS_IVX9 U801 ( .A(OBSERVE_r10[0]), .Z(n316) );
  HS65_GS_OAI222X2 U802 ( .A(n133), .B(n368), .C(n315), .D(n370), .E(n292), 
        .F(n371), .Z(n587) );
  HS65_GS_IVX9 U803 ( .A(OBSERVE_r11[0]), .Z(n315) );
  HS65_GS_OAI222X2 U804 ( .A(n133), .B(n349), .C(n314), .D(n351), .E(n292), 
        .F(n352), .Z(n571) );
  HS65_GS_IVX9 U805 ( .A(OBSERVE_r12[0]), .Z(n314) );
  HS65_GS_OAI222X2 U806 ( .A(n133), .B(n330), .C(n313), .D(n332), .E(n292), 
        .F(n333), .Z(n555) );
  HS65_GS_IVX9 U807 ( .A(OBSERVE_r13[0]), .Z(n313) );
  HS65_GS_OAI222X2 U808 ( .A(n278), .B(n283), .C(n336), .D(n276), .E(n290), 
        .F(n277), .Z(n522) );
  HS65_GS_IVX9 U809 ( .A(OBSERVE_r15[2]), .Z(n336) );
  HS65_GS_OAI222X2 U810 ( .A(n309), .B(n133), .C(n310), .D(n311), .E(n292), 
        .F(n312), .Z(n539) );
  HS65_GS_IVX9 U811 ( .A(OBSERVE_r14[0]), .Z(n310) );
  HS65_GS_OAI222X2 U812 ( .A(n278), .B(n284), .C(n322), .D(n276), .E(n291), 
        .F(n277), .Z(n523) );
  HS65_GS_IVX9 U813 ( .A(OBSERVE_r15[1]), .Z(n322) );
  HS65_GS_OAI222X2 U814 ( .A(n278), .B(n133), .C(n308), .D(n276), .E(n292), 
        .F(n277), .Z(n715) );
  HS65_GS_IVX9 U815 ( .A(OBSERVE_r15[0]), .Z(n308) );
  HS65_GS_OAI222X2 U816 ( .A(n133), .B(n465), .C(n321), .D(n463), .E(n292), 
        .F(n464), .Z(n699) );
  HS65_GS_IVX9 U817 ( .A(OBSERVE_r4[0]), .Z(n321) );
  HS65_GS_OAI222X2 U818 ( .A(n283), .B(n424), .C(n488), .D(n425), .E(n290), 
        .F(n426), .Z(n633) );
  HS65_GS_OAI222X2 U819 ( .A(n284), .B(n424), .C(n489), .D(n425), .E(n291), 
        .F(n426), .Z(n634) );
  HS65_GS_OAI222X2 U820 ( .A(n133), .B(n424), .C(n490), .D(n425), .E(n292), 
        .F(n426), .Z(n635) );
  HS65_GS_OAI222X2 U821 ( .A(n283), .B(n406), .C(n342), .D(n408), .E(n290), 
        .F(n409), .Z(n617) );
  HS65_GS_IVX9 U822 ( .A(OBSERVE_r9[2]), .Z(n342) );
  HS65_GS_OAI222X2 U823 ( .A(n284), .B(n406), .C(n328), .D(n408), .E(n291), 
        .F(n409), .Z(n618) );
  HS65_GS_IVX9 U824 ( .A(OBSERVE_r9[1]), .Z(n328) );
  HS65_GS_OAI222X2 U825 ( .A(n133), .B(n448), .C(n320), .D(n450), .E(n292), 
        .F(n451), .Z(n683) );
  HS65_GS_IVX9 U826 ( .A(OBSERVE_r5[0]), .Z(n320) );
  HS65_GS_OAI222X2 U827 ( .A(n133), .B(n432), .C(n319), .D(n434), .E(n292), 
        .F(n435), .Z(n667) );
  HS65_GS_IVX9 U828 ( .A(OBSERVE_r6[0]), .Z(n319) );
  HS65_GS_OAI222X2 U829 ( .A(n133), .B(n427), .C(n318), .D(n428), .E(n292), 
        .F(n429), .Z(n651) );
  HS65_GS_IVX9 U830 ( .A(OBSERVE_r7[0]), .Z(n318) );
  HS65_GS_OAI222X2 U831 ( .A(n133), .B(n406), .C(n317), .D(n408), .E(n292), 
        .F(n409), .Z(n619) );
  HS65_GS_IVX9 U832 ( .A(OBSERVE_r9[0]), .Z(n317) );
  HS65_GS_IVX9 U833 ( .A(OBSERVE_r4[15]), .Z(n770) );
  HS65_GS_IVX9 U834 ( .A(OBSERVE_r8[15]), .Z(n766) );
  HS65_GS_IVX9 U835 ( .A(OBSERVE_r7[15]), .Z(n767) );
  HS65_GS_IVX9 U836 ( .A(OBSERVE_r3[15]), .Z(n294) );
  HS65_GS_MX41X7 U837 ( .D0(n781), .S0(reg_dest_val[2]), .D1(reg_sp_val[2]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[2]), .D3(n469), .S3(
        OBSERVE_r1[2]), .Z(n713) );
  HS65_GS_MX41X7 U838 ( .D0(n781), .S0(reg_dest_val[1]), .D1(reg_sp_val[1]), 
        .S1(n467), .D2(n468), .S2(reg_incr_val[1]), .D3(n469), .S3(
        OBSERVE_r1[1]), .Z(n714) );
  HS65_GS_AO22X9 U839 ( .A(n259), .B(OBSERVE_r3[0]), .C(n780), .D(
        reg_dest_val[0]), .Z(n491) );
  HS65_GS_AO12X9 U840 ( .A(reg_dest_wr), .B(inst_dest[0]), .C(reg_pc_call), 
        .Z(pc_sw_wr) );
  HS65_GS_BFX9 U841 ( .A(reg_dest_val[7]), .Z(pc_sw[7]) );
  HS65_GS_BFX9 U842 ( .A(reg_dest_val[6]), .Z(pc_sw[6]) );
  HS65_GS_BFX9 U843 ( .A(reg_dest_val[5]), .Z(pc_sw[5]) );
  HS65_GS_BFX9 U844 ( .A(reg_dest_val[4]), .Z(pc_sw[4]) );
  HS65_GS_BFX9 U845 ( .A(reg_dest_val[3]), .Z(pc_sw[3]) );
  HS65_GS_BFX9 U846 ( .A(reg_dest_val[2]), .Z(pc_sw[2]) );
  HS65_GS_BFX9 U847 ( .A(reg_dest_val[1]), .Z(pc_sw[1]) );
  HS65_GS_BFX9 U848 ( .A(reg_dest_val[0]), .Z(pc_sw[0]) );
  HS65_GSS_XOR2X3 U849 ( .A(reg_src[15]), .B(add_155_carry[15]), .Z(
        reg_incr_val[15]) );
  HS65_GS_AND2X4 U850 ( .A(add_155_carry[14]), .B(reg_src[14]), .Z(
        add_155_carry[15]) );
  HS65_GSS_XOR2X3 U851 ( .A(add_155_carry[14]), .B(reg_src[14]), .Z(
        reg_incr_val[14]) );
  HS65_GS_AND2X4 U852 ( .A(add_155_carry[13]), .B(reg_src[13]), .Z(
        add_155_carry[14]) );
  HS65_GSS_XOR2X3 U853 ( .A(add_155_carry[13]), .B(reg_src[13]), .Z(
        reg_incr_val[13]) );
  HS65_GS_AND2X4 U854 ( .A(add_155_carry[12]), .B(reg_src[12]), .Z(
        add_155_carry[13]) );
  HS65_GSS_XOR2X3 U855 ( .A(add_155_carry[12]), .B(reg_src[12]), .Z(
        reg_incr_val[12]) );
  HS65_GS_AND2X4 U856 ( .A(add_155_carry[11]), .B(reg_src[11]), .Z(
        add_155_carry[12]) );
  HS65_GSS_XOR2X3 U857 ( .A(add_155_carry[11]), .B(reg_src[11]), .Z(
        reg_incr_val[11]) );
  HS65_GS_AND2X4 U858 ( .A(add_155_carry[10]), .B(reg_src[10]), .Z(
        add_155_carry[11]) );
  HS65_GSS_XOR2X3 U859 ( .A(add_155_carry[10]), .B(reg_src[10]), .Z(
        reg_incr_val[10]) );
  HS65_GS_AND2X4 U860 ( .A(add_155_carry[9]), .B(reg_src[9]), .Z(
        add_155_carry[10]) );
  HS65_GSS_XOR2X3 U861 ( .A(add_155_carry[9]), .B(reg_src[9]), .Z(
        reg_incr_val[9]) );
  HS65_GS_AND2X4 U862 ( .A(add_155_carry[8]), .B(reg_src[8]), .Z(
        add_155_carry[9]) );
  HS65_GSS_XOR2X3 U863 ( .A(add_155_carry[8]), .B(reg_src[8]), .Z(
        reg_incr_val[8]) );
  HS65_GS_AND2X4 U864 ( .A(add_155_carry[7]), .B(reg_src[7]), .Z(
        add_155_carry[8]) );
  HS65_GSS_XOR2X3 U865 ( .A(add_155_carry[7]), .B(reg_src[7]), .Z(
        reg_incr_val[7]) );
  HS65_GS_AND2X4 U866 ( .A(add_155_carry[6]), .B(reg_src[6]), .Z(
        add_155_carry[7]) );
  HS65_GSS_XOR2X3 U867 ( .A(add_155_carry[6]), .B(reg_src[6]), .Z(
        reg_incr_val[6]) );
  HS65_GS_AND2X4 U868 ( .A(add_155_carry[5]), .B(reg_src[5]), .Z(
        add_155_carry[6]) );
  HS65_GSS_XOR2X3 U869 ( .A(add_155_carry[5]), .B(reg_src[5]), .Z(
        reg_incr_val[5]) );
  HS65_GS_AND2X4 U870 ( .A(add_155_carry[4]), .B(reg_src[4]), .Z(
        add_155_carry[5]) );
  HS65_GSS_XOR2X3 U871 ( .A(add_155_carry[4]), .B(reg_src[4]), .Z(
        reg_incr_val[4]) );
  HS65_GS_AND2X4 U872 ( .A(add_155_carry[3]), .B(reg_src[3]), .Z(
        add_155_carry[4]) );
  HS65_GSS_XOR2X3 U873 ( .A(add_155_carry[3]), .B(reg_src[3]), .Z(
        reg_incr_val[3]) );
  HS65_GS_AND2X4 U874 ( .A(add_155_carry[2]), .B(reg_src[2]), .Z(
        add_155_carry[3]) );
  HS65_GSS_XOR2X3 U875 ( .A(add_155_carry[2]), .B(reg_src[2]), .Z(
        reg_incr_val[2]) );
  HS65_GS_AND2X4 U876 ( .A(reg_src[0]), .B(add_155_B_0_), .Z(add_155_carry[1])
         );
endmodule


module omsp_alu_DW01_add_9 ( A, B, CI, SUM, CO );
  input [16:0] A;
  input [16:0] B;
  output [16:0] SUM;
  input CI;
  output CO;
  wire   n1;
  wire   [15:2] carry;

  HS65_GS_FA1X4 U1_13 ( .A0(A[13]), .B0(B[13]), .CI(carry[13]), .CO(carry[14]), 
        .S0(SUM[13]) );
  HS65_GS_FA1X4 U1_14 ( .A0(A[14]), .B0(B[14]), .CI(carry[14]), .CO(carry[15]), 
        .S0(SUM[14]) );
  HS65_GS_FA1X4 U1_12 ( .A0(A[12]), .B0(B[12]), .CI(carry[12]), .CO(carry[13]), 
        .S0(SUM[12]) );
  HS65_GS_FA1X4 U1_11 ( .A0(A[11]), .B0(B[11]), .CI(carry[11]), .CO(carry[12]), 
        .S0(SUM[11]) );
  HS65_GS_FA1X4 U1_10 ( .A0(A[10]), .B0(B[10]), .CI(carry[10]), .CO(carry[11]), 
        .S0(SUM[10]) );
  HS65_GS_FA1X4 U1_9 ( .A0(A[9]), .B0(B[9]), .CI(carry[9]), .CO(carry[10]), 
        .S0(SUM[9]) );
  HS65_GS_FA1X4 U1_8 ( .A0(A[8]), .B0(B[8]), .CI(carry[8]), .CO(carry[9]), 
        .S0(SUM[8]) );
  HS65_GS_FA1X4 U1_7 ( .A0(A[7]), .B0(B[7]), .CI(carry[7]), .CO(carry[8]), 
        .S0(SUM[7]) );
  HS65_GS_FA1X4 U1_6 ( .A0(A[6]), .B0(B[6]), .CI(carry[6]), .CO(carry[7]), 
        .S0(SUM[6]) );
  HS65_GS_FA1X4 U1_5 ( .A0(A[5]), .B0(B[5]), .CI(carry[5]), .CO(carry[6]), 
        .S0(SUM[5]) );
  HS65_GS_FA1X4 U1_1 ( .A0(A[1]), .B0(B[1]), .CI(n1), .CO(carry[2]), .S0(
        SUM[1]) );
  HS65_GS_FA1X4 U1_2 ( .A0(A[2]), .B0(B[2]), .CI(carry[2]), .CO(carry[3]), 
        .S0(SUM[2]) );
  HS65_GS_FA1X4 U1_4 ( .A0(A[4]), .B0(B[4]), .CI(carry[4]), .CO(carry[5]), 
        .S0(SUM[4]) );
  HS65_GS_FA1X4 U1_3 ( .A0(A[3]), .B0(B[3]), .CI(carry[3]), .CO(carry[4]), 
        .S0(SUM[3]) );
  HS65_GS_FA1X4 U1_15 ( .A0(A[15]), .B0(B[15]), .CI(carry[15]), .CO(SUM[16]), 
        .S0(SUM[15]) );
  HS65_GS_AND2X4 U1 ( .A(A[0]), .B(B[0]), .Z(n1) );
  HS65_GSS_XOR2X6 U2 ( .A(A[0]), .B(B[0]), .Z(SUM[0]) );
endmodule


module omsp_alu ( alu_out, alu_out_add, alu_stat, alu_stat_wr, dbg_halt_st, 
        exec_cycle, inst_alu, inst_bw, inst_jmp, inst_so, op_dst, op_src, 
        status );
  output [15:0] alu_out;
  output [15:0] alu_out_add;
  output [3:0] alu_stat;
  output [3:0] alu_stat_wr;
  input [11:0] inst_alu;
  input [7:0] inst_jmp;
  input [7:0] inst_so;
  input [15:0] op_dst;
  input [15:0] op_src;
  input [3:0] status;
  input dbg_halt_st, exec_cycle, inst_bw;
  wire   alu_inc, alu_dadd2_4_, alu_dadd1_4_, alu_dadd0_4_, alu_add_16_, N54,
         N53, N52, N51, N41, N40, N39, N38, N28, N27, N26, N25, N15, N14, N13,
         N12, n135, n136, n137, n138, n139, n140, n141, n142, n143, n144, n145,
         n146, n147, n148, n149, n150, n151, n152, n153, n154, n155, n156,
         n157, n158, n159, n160, n161, n162, n163, n164, n165, n166, n167,
         n168, n169, n170, n171, n172, n173, n174, n175, n176, n177, n178,
         n179, n180, n181, n182, n183, n184, n185, n186, n187, n188, n189,
         n190, n191, n192, n193, n194, n195, n196, n197, n198, n199, n200,
         n201, n202, n203, n204, n205, n206, n207, n208, n209, n210, n211,
         n212, n213, n214, n215, n216, n217, n218, n219, n220, n221, n222,
         n223, n224, n225, n226, n227, n228, n229, n230, n231, n232, n233,
         n234, n235, n236, n237, n238, n239, n240, n241, n242, n243, n244,
         n245, n246, n247, n248, n249, n250, n251, n252, n253, n254, n255,
         n256, n257, n258, n259, n260, n261, n262, n263, n264, n265, n266,
         n267, n268, n269, n270, n271, add_1_root_add_100_2_C188_B_3_,
         add_1_root_add_100_2_C188_A_0_, add_1_root_add_100_2_C188_A_1_,
         add_1_root_add_100_2_C188_A_2_, add_1_root_add_100_2_C187_A_0_,
         add_1_root_add_100_2_C187_A_1_, add_1_root_add_100_2_C187_A_2_,
         add_1_root_add_100_2_C187_A_3_, add_1_root_add_100_2_C186_A_0_,
         add_1_root_add_100_2_C186_A_1_, add_1_root_add_100_2_C186_A_2_,
         add_1_root_add_100_2_C186_A_3_, add_1_root_add_100_2_C185_A_0_,
         add_1_root_add_100_2_C185_A_1_, add_1_root_add_100_2_C185_A_2_,
         add_1_root_add_100_2_C185_A_3_, n1, n2, n3, n4, n5, n9, n10, n11, n12,
         n13, n14, n15, n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26,
         n27, n28, n29, n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40,
         n41, n42, n43, n44, n45, n46, n47;
  wire   [15:0] op_src_in_jmp;
  wire   [16:0] alu_add_inc;
  wire   [3:0] alu_dadd0;
  wire   [3:0] alu_dadd1;
  wire   [3:0] alu_dadd2;
  wire   [4:0] alu_dadd3;
  wire   [4:2] add_102_C188_aco_carry;
  wire   [3:1] add_1_root_add_100_2_C188_carry;
  wire   [4:2] add_102_C187_aco_carry;
  wire   [3:1] add_1_root_add_100_2_C187_carry;
  wire   [4:2] add_102_C186_aco_carry;
  wire   [3:1] add_1_root_add_100_2_C186_carry;
  wire   [4:2] add_102_C185_aco_carry;
  wire   [3:1] add_1_root_add_100_2_C185_carry;
  wire   [16:1] add_180_carry;

  omsp_alu_DW01_add_9 add_171 ( .A({1'b0, op_src_in_jmp}), .B({1'b0, 
        add_1_root_add_100_2_C188_B_3_, n22, n24, n26, n28, n30, n32, n12, 
        op_dst[7:0]}), .CI(1'b0), .SUM({alu_add_16_, alu_out_add}) );
  HS65_GS_AND2X4 U3 ( .A(inst_alu[0]), .B(exec_cycle), .Z(n194) );
  HS65_GS_NOR2AX3 U4 ( .A(inst_alu[7]), .B(n174), .Z(n173) );
  HS65_GS_CB4I6X9 U5 ( .A(N39), .B(N38), .C(N40), .D(N41), .Z(n1) );
  HS65_GS_CB4I6X9 U6 ( .A(N52), .B(N51), .C(N53), .D(N54), .Z(n2) );
  HS65_GS_FA1X4 U7 ( .A0(N39), .B0(n1), .CI(add_102_C187_aco_carry[2]), .CO(
        add_102_C187_aco_carry[3]), .S0(alu_dadd2[2]) );
  HS65_GS_FA1X4 U8 ( .A0(N52), .B0(n2), .CI(add_102_C188_aco_carry[2]), .CO(
        add_102_C188_aco_carry[3]), .S0(alu_dadd3[2]) );
  HS65_GS_FA1X4 U9 ( .A0(add_1_root_add_100_2_C187_A_0_), .B0(n12), .CI(
        alu_dadd1_4_), .CO(add_1_root_add_100_2_C187_carry[1]), .S0(
        alu_dadd2[0]) );
  HS65_GS_IVX9 U10 ( .A(n137), .Z(add_1_root_add_100_2_C187_A_0_) );
  HS65_GS_IVX9 U11 ( .A(n163), .Z(alu_out[15]) );
  HS65_GS_CB4I6X9 U12 ( .A(N13), .B(N12), .C(N14), .D(N15), .Z(n3) );
  HS65_GS_CB4I6X9 U13 ( .A(N26), .B(N25), .C(N27), .D(N28), .Z(n4) );
  HS65_GS_FA1X4 U14 ( .A0(add_1_root_add_100_2_C187_A_2_), .B0(n30), .CI(
        add_1_root_add_100_2_C187_carry[2]), .CO(
        add_1_root_add_100_2_C187_carry[3]), .S0(N39) );
  HS65_GS_IVX9 U15 ( .A(n150), .Z(add_1_root_add_100_2_C187_A_2_) );
  HS65_GS_FA1X4 U16 ( .A0(add_1_root_add_100_2_C187_A_1_), .B0(n32), .CI(
        add_1_root_add_100_2_C187_carry[1]), .CO(
        add_1_root_add_100_2_C187_carry[2]), .S0(N38) );
  HS65_GS_IVX9 U17 ( .A(n136), .Z(add_1_root_add_100_2_C187_A_1_) );
  HS65_GS_FA1X4 U18 ( .A0(add_1_root_add_100_2_C188_A_1_), .B0(n24), .CI(
        add_1_root_add_100_2_C188_carry[1]), .CO(
        add_1_root_add_100_2_C188_carry[2]), .S0(N51) );
  HS65_GS_IVX9 U19 ( .A(n147), .Z(add_1_root_add_100_2_C188_A_1_) );
  HS65_GS_FA1X4 U20 ( .A0(add_1_root_add_100_2_C188_A_2_), .B0(n22), .CI(
        add_1_root_add_100_2_C188_carry[2]), .CO(
        add_1_root_add_100_2_C188_carry[3]), .S0(N52) );
  HS65_GS_IVX9 U21 ( .A(n146), .Z(add_1_root_add_100_2_C188_A_2_) );
  HS65_GS_FA1X4 U22 ( .A0(add_1_root_add_100_2_C188_A_0_), .B0(n26), .CI(
        alu_dadd2_4_), .CO(add_1_root_add_100_2_C188_carry[1]), .S0(
        alu_dadd3[0]) );
  HS65_GS_IVX9 U23 ( .A(n148), .Z(add_1_root_add_100_2_C188_A_0_) );
  HS65_GS_FA1X4 U25 ( .A0(n21), .B0(add_1_root_add_100_2_C188_B_3_), .CI(
        add_1_root_add_100_2_C188_carry[3]), .CO(N54), .S0(N53) );
  HS65_GS_FA1X4 U26 ( .A0(add_1_root_add_100_2_C187_A_3_), .B0(n28), .CI(
        add_1_root_add_100_2_C187_carry[3]), .CO(N41), .S0(N40) );
  HS65_GS_IVX9 U27 ( .A(n149), .Z(add_1_root_add_100_2_C187_A_3_) );
  HS65_GS_FA1X4 U28 ( .A0(N13), .B0(n3), .CI(add_102_C185_aco_carry[2]), .CO(
        add_102_C185_aco_carry[3]), .S0(alu_dadd0[2]) );
  HS65_GS_FA1X4 U29 ( .A0(N26), .B0(n4), .CI(add_102_C186_aco_carry[2]), .CO(
        add_102_C186_aco_carry[3]), .S0(alu_dadd1[2]) );
  HS65_GS_IVX9 U30 ( .A(n171), .Z(alu_out[8]) );
  HS65_GS_IVX9 U31 ( .A(n183), .Z(alu_out[11]) );
  HS65_GS_IVX9 U32 ( .A(n182), .Z(alu_out[10]) );
  HS65_GS_NAND4ABX3 U34 ( .A(alu_out[9]), .B(alu_out[14]), .C(n171), .D(n163), 
        .Z(n180) );
  HS65_GS_IVX9 U35 ( .A(n166), .Z(alu_out[7]) );
  HS65_GS_IVX9 U36 ( .A(n179), .Z(alu_out[5]) );
  HS65_GS_NAND4ABX3 U37 ( .A(alu_out[13]), .B(alu_out[12]), .C(n182), .D(n183), 
        .Z(n181) );
  HS65_GS_IVX9 U38 ( .A(n167), .Z(alu_stat[1]) );
  HS65_GS_IVX9 U39 ( .A(n145), .Z(n21) );
  HS65_GS_FA1X4 U40 ( .A0(add_1_root_add_100_2_C186_A_0_), .B0(op_dst[4]), 
        .CI(alu_dadd0_4_), .CO(add_1_root_add_100_2_C186_carry[1]), .S0(
        alu_dadd1[0]) );
  HS65_GS_IVX9 U41 ( .A(n141), .Z(add_1_root_add_100_2_C186_A_0_) );
  HS65_GSS_XOR2X6 U42 ( .A(n19), .B(n194), .Z(n151) );
  HS65_GS_AOI222X2 U43 ( .A(alu_dadd3[3]), .B(n173), .C(n184), .D(n231), .E(
        alu_add_inc[15]), .F(n174), .Z(n163) );
  HS65_GS_OAI212X5 U44 ( .A(n206), .B(n43), .C(n35), .D(n41), .E(n232), .Z(
        n231) );
  HS65_GSS_XOR2X6 U45 ( .A(n35), .B(n194), .Z(n138) );
  HS65_GSS_XOR2X6 U46 ( .A(n36), .B(n194), .Z(n139) );
  HS65_GSS_XOR2X6 U47 ( .A(n37), .B(n194), .Z(n140) );
  HS65_GSS_XOR2X6 U48 ( .A(n38), .B(n194), .Z(n141) );
  HS65_GSS_XOR2X6 U49 ( .A(n13), .B(n194), .Z(n142) );
  HS65_GSS_XOR2X6 U50 ( .A(n15), .B(n194), .Z(n143) );
  HS65_GSS_XOR2X6 U51 ( .A(n17), .B(n194), .Z(n144) );
  HS65_GS_NOR2X6 U52 ( .A(n135), .B(n145), .Z(op_src_in_jmp[15]) );
  HS65_GS_IVX9 U53 ( .A(op_src[7]), .Z(n35) );
  HS65_GS_NOR2X6 U54 ( .A(n151), .B(n135), .Z(op_src_in_jmp[0]) );
  HS65_GS_NAND2X7 U55 ( .A(n235), .B(n192), .Z(n145) );
  HS65_GSS_XOR2X6 U56 ( .A(op_src[15]), .B(n194), .Z(n235) );
  HS65_GS_IVX9 U57 ( .A(op_src[6]), .Z(n36) );
  HS65_GS_IVX9 U58 ( .A(op_src[5]), .Z(n37) );
  HS65_GS_IVX9 U59 ( .A(op_src[4]), .Z(n38) );
  HS65_GS_IVX9 U60 ( .A(op_src[3]), .Z(n13) );
  HS65_GS_IVX9 U61 ( .A(op_src[2]), .Z(n15) );
  HS65_GS_IVX9 U62 ( .A(op_src[1]), .Z(n17) );
  HS65_GS_NAND2X7 U63 ( .A(n266), .B(n192), .Z(n150) );
  HS65_GSS_XOR2X6 U64 ( .A(op_src[10]), .B(n194), .Z(n266) );
  HS65_GS_NAND2X7 U65 ( .A(n242), .B(n192), .Z(n146) );
  HS65_GSS_XOR2X6 U66 ( .A(op_src[14]), .B(n194), .Z(n242) );
  HS65_GS_NAND2X7 U67 ( .A(n193), .B(n192), .Z(n136) );
  HS65_GSS_XOR2X6 U68 ( .A(op_src[9]), .B(n194), .Z(n193) );
  HS65_GS_NAND2X7 U69 ( .A(n248), .B(n192), .Z(n147) );
  HS65_GSS_XOR2X6 U70 ( .A(op_src[13]), .B(n194), .Z(n248) );
  HS65_GS_NAND2X7 U71 ( .A(n254), .B(n192), .Z(n148) );
  HS65_GSS_XOR2X6 U72 ( .A(op_src[12]), .B(n194), .Z(n254) );
  HS65_GS_NAND2X7 U73 ( .A(n260), .B(n192), .Z(n149) );
  HS65_GSS_XOR2X6 U74 ( .A(op_src[11]), .B(n194), .Z(n260) );
  HS65_GS_NAND2X7 U75 ( .A(n200), .B(n192), .Z(n137) );
  HS65_GSS_XOR2X6 U76 ( .A(op_src[8]), .B(n194), .Z(n200) );
  HS65_GS_IVX9 U77 ( .A(n162), .Z(add_1_root_add_100_2_C188_B_3_) );
  HS65_GS_NOR2X6 U78 ( .A(n142), .B(n135), .Z(op_src_in_jmp[3]) );
  HS65_GS_NOR2X6 U79 ( .A(n141), .B(n135), .Z(op_src_in_jmp[4]) );
  HS65_GS_NOR2X6 U80 ( .A(n143), .B(n135), .Z(op_src_in_jmp[2]) );
  HS65_GS_NOR2X6 U81 ( .A(n144), .B(n135), .Z(op_src_in_jmp[1]) );
  HS65_GS_NOR2X6 U82 ( .A(n140), .B(n135), .Z(op_src_in_jmp[5]) );
  HS65_GS_NOR2X6 U83 ( .A(n139), .B(n135), .Z(op_src_in_jmp[6]) );
  HS65_GS_NOR2X6 U84 ( .A(n138), .B(n135), .Z(op_src_in_jmp[7]) );
  HS65_GS_NOR2X6 U85 ( .A(n135), .B(n137), .Z(op_src_in_jmp[8]) );
  HS65_GS_NOR2X6 U86 ( .A(n135), .B(n136), .Z(op_src_in_jmp[9]) );
  HS65_GS_NOR2X6 U87 ( .A(n135), .B(n150), .Z(op_src_in_jmp[10]) );
  HS65_GS_NOR2X6 U88 ( .A(n135), .B(n149), .Z(op_src_in_jmp[11]) );
  HS65_GS_NOR2X6 U89 ( .A(n135), .B(n148), .Z(op_src_in_jmp[12]) );
  HS65_GS_NOR2X6 U90 ( .A(n135), .B(n146), .Z(op_src_in_jmp[14]) );
  HS65_GS_NOR2X6 U91 ( .A(n135), .B(n147), .Z(op_src_in_jmp[13]) );
  HS65_GS_FA1X4 U92 ( .A0(add_1_root_add_100_2_C185_A_2_), .B0(op_dst[2]), 
        .CI(add_1_root_add_100_2_C185_carry[2]), .CO(
        add_1_root_add_100_2_C185_carry[3]), .S0(N13) );
  HS65_GS_IVX9 U93 ( .A(n143), .Z(add_1_root_add_100_2_C185_A_2_) );
  HS65_GS_FA1X4 U94 ( .A0(add_1_root_add_100_2_C185_A_1_), .B0(op_dst[1]), 
        .CI(add_1_root_add_100_2_C185_carry[1]), .CO(
        add_1_root_add_100_2_C185_carry[2]), .S0(N12) );
  HS65_GS_IVX9 U95 ( .A(n144), .Z(add_1_root_add_100_2_C185_A_1_) );
  HS65_GS_FA1X4 U96 ( .A0(add_1_root_add_100_2_C186_A_1_), .B0(op_dst[5]), 
        .CI(add_1_root_add_100_2_C186_carry[1]), .CO(
        add_1_root_add_100_2_C186_carry[2]), .S0(N25) );
  HS65_GS_IVX9 U97 ( .A(n140), .Z(add_1_root_add_100_2_C186_A_1_) );
  HS65_GS_FA1X4 U98 ( .A0(add_1_root_add_100_2_C186_A_2_), .B0(op_dst[6]), 
        .CI(add_1_root_add_100_2_C186_carry[2]), .CO(
        add_1_root_add_100_2_C186_carry[3]), .S0(N26) );
  HS65_GS_IVX9 U99 ( .A(n139), .Z(add_1_root_add_100_2_C186_A_2_) );
  HS65_GS_FA1X4 U100 ( .A0(add_1_root_add_100_2_C186_A_3_), .B0(op_dst[7]), 
        .CI(add_1_root_add_100_2_C186_carry[3]), .CO(N28), .S0(N27) );
  HS65_GS_IVX9 U101 ( .A(n138), .Z(add_1_root_add_100_2_C186_A_3_) );
  HS65_GS_FA1X4 U102 ( .A0(add_1_root_add_100_2_C185_A_3_), .B0(op_dst[3]), 
        .CI(add_1_root_add_100_2_C185_carry[3]), .CO(N15), .S0(N14) );
  HS65_GS_IVX9 U103 ( .A(n142), .Z(add_1_root_add_100_2_C185_A_3_) );
  HS65_GS_AOI222X2 U104 ( .A(alu_dadd1[3]), .B(n173), .C(n184), .D(n201), .E(
        alu_add_inc[7]), .F(n174), .Z(n166) );
  HS65_GS_OAI212X5 U105 ( .A(n202), .B(n34), .C(n20), .D(n41), .E(n203), .Z(
        n201) );
  HS65_GS_AOI222X2 U106 ( .A(alu_dadd2[0]), .B(n173), .C(n184), .D(n195), .E(
        alu_add_inc[8]), .F(n174), .Z(n171) );
  HS65_GS_OAI212X5 U107 ( .A(n33), .B(n43), .C(n19), .D(n41), .E(n196), .Z(
        n195) );
  HS65_GS_AOI212X4 U108 ( .A(add_1_root_add_100_2_C187_A_0_), .B(n197), .C(n12), .D(n198), .E(n189), .Z(n196) );
  HS65_GS_AOI222X2 U109 ( .A(alu_dadd2[2]), .B(n173), .C(n184), .D(n261), .E(
        alu_add_inc[10]), .F(n174), .Z(n182) );
  HS65_GS_OAI212X5 U110 ( .A(n29), .B(n43), .C(n15), .D(n41), .E(n262), .Z(
        n261) );
  HS65_GS_AOI212X4 U111 ( .A(add_1_root_add_100_2_C187_A_2_), .B(n263), .C(n30), .D(n264), .E(n189), .Z(n262) );
  HS65_GS_AOI222X2 U112 ( .A(alu_dadd1[1]), .B(n173), .C(n184), .D(n211), .E(
        alu_add_inc[5]), .F(n174), .Z(n179) );
  HS65_GS_OAI212X5 U113 ( .A(n25), .B(n41), .C(n37), .D(n40), .E(n212), .Z(
        n211) );
  HS65_GS_AOI222X2 U114 ( .A(alu_dadd2[3]), .B(n173), .C(n184), .D(n255), .E(
        alu_add_inc[11]), .F(n174), .Z(n183) );
  HS65_GS_OAI212X5 U115 ( .A(n27), .B(n43), .C(n13), .D(n41), .E(n256), .Z(
        n255) );
  HS65_GS_AO222X4 U116 ( .A(alu_dadd2[1]), .B(n173), .C(n184), .D(n185), .E(
        alu_add_inc[9]), .F(n174), .Z(alu_out[9]) );
  HS65_GS_OAI212X5 U117 ( .A(n31), .B(n43), .C(n17), .D(n41), .E(n186), .Z(
        n185) );
  HS65_GS_NAND4ABX3 U118 ( .A(alu_out[2]), .B(alu_out[1]), .C(n175), .D(n176), 
        .Z(n167) );
  HS65_GS_NOR3X4 U119 ( .A(n177), .B(alu_out[4]), .C(alu_out[3]), .Z(n176) );
  HS65_GS_CBI4I6X5 U120 ( .A(n180), .B(n181), .C(n39), .D(alu_out[0]), .Z(n175) );
  HS65_GS_NAND3X5 U121 ( .A(n178), .B(n166), .C(n179), .Z(n177) );
  HS65_GS_NOR4ABX2 U122 ( .A(n161), .B(n162), .C(n21), .D(n163), .Z(n160) );
  HS65_GS_AO222X4 U123 ( .A(alu_dadd3[1]), .B(n173), .C(n184), .D(n243), .E(
        alu_add_inc[13]), .F(n174), .Z(alu_out[13]) );
  HS65_GS_OAI212X5 U124 ( .A(n23), .B(n43), .C(n37), .D(n41), .E(n244), .Z(
        n243) );
  HS65_GS_AO222X4 U125 ( .A(alu_dadd3[2]), .B(n173), .C(n184), .D(n237), .E(
        alu_add_inc[14]), .F(n174), .Z(alu_out[14]) );
  HS65_GS_OAI212X5 U126 ( .A(n20), .B(n43), .C(n36), .D(n41), .E(n238), .Z(
        n237) );
  HS65_GS_AOI212X4 U127 ( .A(add_1_root_add_100_2_C188_A_2_), .B(n239), .C(n22), .D(n240), .E(n189), .Z(n238) );
  HS65_GS_AO222X4 U128 ( .A(alu_dadd3[0]), .B(n173), .C(n184), .D(n249), .E(
        alu_add_inc[12]), .F(n174), .Z(alu_out[12]) );
  HS65_GS_OAI212X5 U129 ( .A(n25), .B(n43), .C(n38), .D(n41), .E(n250), .Z(
        n249) );
  HS65_GS_AOI212X4 U130 ( .A(add_1_root_add_100_2_C188_A_0_), .B(n251), .C(n26), .D(n252), .E(n189), .Z(n250) );
  HS65_GS_CBI4I6X5 U131 ( .A(alu_out[15]), .B(n44), .C(n164), .D(n145), .Z(
        n159) );
  HS65_GS_CBI4I1X5 U132 ( .A(n156), .B(n157), .C(n39), .D(n158), .Z(
        alu_stat[3]) );
  HS65_GS_NAND4ABX3 U133 ( .A(n44), .B(op_dst[7]), .C(n138), .D(alu_out[7]), 
        .Z(n157) );
  HS65_GS_NAND3X5 U134 ( .A(add_1_root_add_100_2_C186_A_3_), .B(n165), .C(
        op_dst[7]), .Z(n156) );
  HS65_GS_CBI4I1X5 U135 ( .A(n159), .B(add_1_root_add_100_2_C188_B_3_), .C(
        n160), .D(n39), .Z(n158) );
  HS65_GS_IVX9 U136 ( .A(n178), .Z(alu_out[6]) );
  HS65_GS_OAI21X3 U137 ( .A(n151), .B(n43), .C(n168), .Z(alu_stat[0]) );
  HS65_GS_AOI32X5 U138 ( .A(n167), .B(n43), .C(n169), .D(n161), .E(n170), .Z(
        n168) );
  HS65_GS_OAI22X6 U139 ( .A(n171), .B(n39), .C(inst_bw), .D(n172), .Z(n170) );
  HS65_GS_AOI22X6 U140 ( .A(alu_dadd3[4]), .B(n173), .C(alu_add_inc[16]), .D(
        n174), .Z(n172) );
  HS65_GS_AO222X4 U141 ( .A(alu_dadd0[3]), .B(n173), .C(n184), .D(n219), .E(
        alu_add_inc[3]), .F(n174), .Z(alu_out[3]) );
  HS65_GS_OAI212X5 U142 ( .A(n29), .B(n41), .C(n13), .D(n40), .E(n220), .Z(
        n219) );
  HS65_GS_OAI212X5 U143 ( .A(add_1_root_add_100_2_C186_A_2_), .B(n9), .C(n139), 
        .D(n47), .E(n46), .Z(n209) );
  HS65_GS_OAI212X5 U144 ( .A(add_1_root_add_100_2_C186_A_0_), .B(n9), .C(n141), 
        .D(n47), .E(n46), .Z(n217) );
  HS65_GS_OAI212X5 U145 ( .A(add_1_root_add_100_2_C185_A_2_), .B(n9), .C(n143), 
        .D(n47), .E(n46), .Z(n225) );
  HS65_GS_OAI212X5 U146 ( .A(n22), .B(n9), .C(n47), .D(n241), .E(n191), .Z(
        n239) );
  HS65_GS_OAI212X5 U147 ( .A(n26), .B(n9), .C(n47), .D(n253), .E(n191), .Z(
        n251) );
  HS65_GS_OAI212X5 U148 ( .A(n30), .B(n9), .C(n47), .D(n265), .E(n191), .Z(
        n263) );
  HS65_GS_OAI212X5 U149 ( .A(n12), .B(n9), .C(n47), .D(n199), .E(n191), .Z(
        n197) );
  HS65_GS_AOI212X4 U150 ( .A(add_1_root_add_100_2_C187_A_1_), .B(n187), .C(n32), .D(n188), .E(n189), .Z(n186) );
  HS65_GS_OAI21X3 U151 ( .A(add_1_root_add_100_2_C187_A_1_), .B(n45), .C(n46), 
        .Z(n188) );
  HS65_GS_OAI212X5 U152 ( .A(n32), .B(n9), .C(n47), .D(n190), .E(n191), .Z(
        n187) );
  HS65_GS_AOI212X4 U153 ( .A(add_1_root_add_100_2_C188_A_1_), .B(n245), .C(n24), .D(n246), .E(n189), .Z(n244) );
  HS65_GS_OAI21X3 U154 ( .A(add_1_root_add_100_2_C188_A_1_), .B(n9), .C(n46), 
        .Z(n246) );
  HS65_GS_OAI212X5 U155 ( .A(n24), .B(n9), .C(n47), .D(n247), .E(n191), .Z(
        n245) );
  HS65_GS_AOI212X4 U156 ( .A(add_1_root_add_100_2_C187_A_3_), .B(n257), .C(n28), .D(n258), .E(n189), .Z(n256) );
  HS65_GS_OAI21X3 U157 ( .A(add_1_root_add_100_2_C187_A_3_), .B(n9), .C(n46), 
        .Z(n258) );
  HS65_GS_OAI212X5 U158 ( .A(n28), .B(n9), .C(n47), .D(n259), .E(n191), .Z(
        n257) );
  HS65_GS_AOI212X4 U159 ( .A(n21), .B(n233), .C(add_1_root_add_100_2_C188_B_3_), .D(n234), .E(n189), .Z(n232) );
  HS65_GS_OAI21X3 U160 ( .A(n21), .B(n45), .C(n46), .Z(n234) );
  HS65_GS_OAI212X5 U161 ( .A(add_1_root_add_100_2_C188_B_3_), .B(n9), .C(n162), 
        .D(n47), .E(n191), .Z(n233) );
  HS65_GS_AO222X4 U162 ( .A(alu_dadd0[1]), .B(n173), .C(n184), .D(n227), .E(
        alu_add_inc[1]), .F(n174), .Z(alu_out[1]) );
  HS65_GS_OAI212X5 U163 ( .A(n33), .B(n41), .C(n17), .D(n40), .E(n228), .Z(
        n227) );
  HS65_GS_NOR2X6 U164 ( .A(n40), .B(n35), .Z(n189) );
  HS65_GS_CBI4I6X5 U165 ( .A(op_dst[6]), .B(n45), .C(n191), .D(n139), .Z(n210)
         );
  HS65_GS_CBI4I6X5 U166 ( .A(op_dst[4]), .B(n45), .C(n191), .D(n141), .Z(n218)
         );
  HS65_GS_CBI4I6X5 U167 ( .A(op_dst[2]), .B(n45), .C(n191), .D(n143), .Z(n226)
         );
  HS65_GS_NAND2X7 U168 ( .A(inst_bw), .B(exec_cycle), .Z(n192) );
  HS65_GS_OAI21X3 U169 ( .A(alu_out[7]), .B(n44), .C(n164), .Z(n165) );
  HS65_GS_OAI21X3 U170 ( .A(add_1_root_add_100_2_C188_A_2_), .B(n9), .C(n46), 
        .Z(n240) );
  HS65_GS_OAI21X3 U171 ( .A(add_1_root_add_100_2_C188_A_0_), .B(n45), .C(n46), 
        .Z(n252) );
  HS65_GS_OAI21X3 U172 ( .A(add_1_root_add_100_2_C187_A_2_), .B(n9), .C(n46), 
        .Z(n264) );
  HS65_GS_OAI21X3 U173 ( .A(add_1_root_add_100_2_C187_A_0_), .B(n45), .C(n46), 
        .Z(n198) );
  HS65_GS_IVX9 U174 ( .A(op_src[15]), .Z(n20) );
  HS65_GS_IVX9 U175 ( .A(op_src[10]), .Z(n31) );
  HS65_GS_IVX9 U176 ( .A(op_src[12]), .Z(n27) );
  HS65_GS_IVX9 U177 ( .A(op_src[14]), .Z(n23) );
  HS65_GS_IVX9 U178 ( .A(op_src[13]), .Z(n25) );
  HS65_GS_IVX9 U179 ( .A(op_src[11]), .Z(n29) );
  HS65_GS_IVX9 U180 ( .A(op_src[9]), .Z(n33) );
  HS65_GS_IVX9 U181 ( .A(op_src[8]), .Z(n11) );
  HS65_GS_IVX9 U182 ( .A(n190), .Z(n32) );
  HS65_GS_IVX9 U183 ( .A(n247), .Z(n24) );
  HS65_GS_IVX9 U184 ( .A(n259), .Z(n28) );
  HS65_GS_IVX9 U185 ( .A(n241), .Z(n22) );
  HS65_GS_IVX9 U186 ( .A(n253), .Z(n26) );
  HS65_GS_IVX9 U187 ( .A(n265), .Z(n30) );
  HS65_GS_IVX9 U188 ( .A(n199), .Z(n12) );
  HS65_GS_IVX9 U189 ( .A(op_dst[7]), .Z(n34) );
  HS65_GS_OAI212X5 U190 ( .A(add_1_root_add_100_2_C185_A_0_), .B(n9), .C(n151), 
        .D(n47), .E(n46), .Z(n269) );
  HS65_GS_CBI4I6X5 U191 ( .A(op_dst[0]), .B(n45), .C(n191), .D(n151), .Z(n270)
         );
  HS65_GS_IVX9 U192 ( .A(inst_bw), .Z(n39) );
  HS65_GS_BFX9 U193 ( .A(n45), .Z(n9) );
  HS65_GS_IVX9 U194 ( .A(n161), .Z(n44) );
  HS65_GS_IVX9 U195 ( .A(alu_stat_wr[2]), .Z(n5) );
  HS65_GS_NAND2X7 U196 ( .A(op_dst[15]), .B(n192), .Z(n162) );
  HS65_GS_IVX9 U197 ( .A(op_src[0]), .Z(n19) );
  HS65_GS_FA1X4 U198 ( .A0(add_1_root_add_100_2_C185_A_0_), .B0(op_dst[0]), 
        .CI(status[0]), .CO(add_1_root_add_100_2_C185_carry[1]), .S0(
        alu_dadd0[0]) );
  HS65_GS_IVX9 U199 ( .A(n151), .Z(add_1_root_add_100_2_C185_A_0_) );
  HS65_GS_NAND2X7 U200 ( .A(op_dst[14]), .B(n192), .Z(n241) );
  HS65_GS_NAND2X7 U201 ( .A(op_dst[9]), .B(n192), .Z(n190) );
  HS65_GS_NAND2X7 U202 ( .A(op_dst[13]), .B(n192), .Z(n247) );
  HS65_GS_NAND2X7 U203 ( .A(op_dst[12]), .B(n192), .Z(n253) );
  HS65_GS_NAND2X7 U204 ( .A(op_dst[11]), .B(n192), .Z(n259) );
  HS65_GS_NAND2X7 U205 ( .A(op_dst[10]), .B(n192), .Z(n265) );
  HS65_GS_NAND2X7 U206 ( .A(op_dst[8]), .B(n192), .Z(n199) );
  HS65_GS_AOI222X2 U207 ( .A(alu_dadd1[2]), .B(n173), .C(n184), .D(n207), .E(
        alu_add_inc[6]), .F(n174), .Z(n178) );
  HS65_GS_OAI212X5 U208 ( .A(n23), .B(n41), .C(n36), .D(n40), .E(n208), .Z(
        n207) );
  HS65_GS_AOI212X4 U209 ( .A(inst_alu[10]), .B(op_src[7]), .C(op_dst[6]), .D(
        n209), .E(n210), .Z(n208) );
  HS65_GS_OAI22X6 U210 ( .A(n166), .B(n39), .C(inst_bw), .D(n163), .Z(
        alu_stat[2]) );
  HS65_GS_NAND2X7 U211 ( .A(n152), .B(n153), .Z(n135) );
  HS65_GS_AOI222X2 U212 ( .A(inst_jmp[3]), .B(n18), .C(inst_jmp[1]), .D(n16), 
        .E(status[0]), .F(inst_jmp[2]), .Z(n152) );
  HS65_GS_AOI222X2 U213 ( .A(status[1]), .B(inst_jmp[0]), .C(status[2]), .D(
        n154), .E(n155), .F(n14), .Z(n153) );
  HS65_GS_IVX9 U214 ( .A(status[1]), .Z(n16) );
  HS65_GS_AOI212X4 U215 ( .A(inst_alu[10]), .B(n204), .C(
        add_1_root_add_100_2_C186_A_3_), .D(n205), .E(n189), .Z(n203) );
  HS65_GS_OAI21X3 U216 ( .A(op_dst[7]), .B(n45), .C(n191), .Z(n205) );
  HS65_GS_OAI22X6 U217 ( .A(inst_bw), .B(n11), .C(n206), .D(n39), .Z(n204) );
  HS65_GS_AOI212X4 U218 ( .A(inst_alu[10]), .B(op_src[6]), .C(op_dst[5]), .D(
        n213), .E(n214), .Z(n212) );
  HS65_GS_CBI4I6X5 U219 ( .A(op_dst[5]), .B(n45), .C(n191), .D(n140), .Z(n214)
         );
  HS65_GS_OAI212X5 U220 ( .A(add_1_root_add_100_2_C186_A_1_), .B(n9), .C(n140), 
        .D(n47), .E(n46), .Z(n213) );
  HS65_GS_AOI212X4 U221 ( .A(inst_alu[10]), .B(op_src[4]), .C(op_dst[3]), .D(
        n221), .E(n222), .Z(n220) );
  HS65_GS_CBI4I6X5 U222 ( .A(op_dst[3]), .B(n45), .C(n191), .D(n142), .Z(n222)
         );
  HS65_GS_OAI212X5 U223 ( .A(add_1_root_add_100_2_C185_A_3_), .B(n9), .C(n142), 
        .D(n47), .E(n46), .Z(n221) );
  HS65_GS_AOI212X4 U224 ( .A(inst_alu[10]), .B(op_src[2]), .C(op_dst[1]), .D(
        n229), .E(n230), .Z(n228) );
  HS65_GS_CBI4I6X5 U225 ( .A(op_dst[1]), .B(n45), .C(n191), .D(n144), .Z(n230)
         );
  HS65_GS_OAI212X5 U226 ( .A(add_1_root_add_100_2_C185_A_1_), .B(n9), .C(n144), 
        .D(n47), .E(n46), .Z(n229) );
  HS65_GS_AOI212X4 U227 ( .A(inst_alu[4]), .B(add_1_root_add_100_2_C186_A_3_), 
        .C(inst_alu[6]), .D(n138), .E(inst_alu[5]), .Z(n202) );
  HS65_GS_AO222X4 U228 ( .A(alu_dadd0[2]), .B(n173), .C(n184), .D(n223), .E(
        alu_add_inc[2]), .F(n174), .Z(alu_out[2]) );
  HS65_GS_OAI212X5 U229 ( .A(n31), .B(n41), .C(n15), .D(n40), .E(n224), .Z(
        n223) );
  HS65_GS_AOI212X4 U230 ( .A(inst_alu[10]), .B(op_src[3]), .C(op_dst[2]), .D(
        n225), .E(n226), .Z(n224) );
  HS65_GS_AO222X4 U231 ( .A(alu_dadd1[0]), .B(n173), .C(n184), .D(n215), .E(
        alu_add_inc[4]), .F(n174), .Z(alu_out[4]) );
  HS65_GS_OAI212X5 U232 ( .A(n27), .B(n41), .C(n38), .D(n40), .E(n216), .Z(
        n215) );
  HS65_GS_AOI212X4 U233 ( .A(inst_alu[10]), .B(op_src[5]), .C(op_dst[4]), .D(
        n217), .E(n218), .Z(n216) );
  HS65_GS_IVX9 U234 ( .A(status[0]), .Z(n18) );
  HS65_GS_OA22X9 U235 ( .A(n18), .B(n42), .C(inst_so[0]), .D(n236), .Z(n206)
         );
  HS65_GS_IVX9 U236 ( .A(inst_so[0]), .Z(n42) );
  HS65_GS_AOI22X6 U237 ( .A(op_src[15]), .B(n39), .C(inst_bw), .D(op_src[7]), 
        .Z(n236) );
  HS65_GS_IVX9 U238 ( .A(status[2]), .Z(n14) );
  HS65_GS_IVX9 U239 ( .A(status[3]), .Z(n10) );
  HS65_GS_AO212X4 U240 ( .A(status[3]), .B(inst_jmp[5]), .C(inst_jmp[6]), .D(
        n10), .E(inst_jmp[4]), .Z(n155) );
  HS65_GS_AO22X9 U241 ( .A(inst_jmp[6]), .B(status[3]), .C(n10), .D(
        inst_jmp[5]), .Z(n154) );
  HS65_GS_AO222X4 U242 ( .A(alu_dadd0[0]), .B(n173), .C(n184), .D(n267), .E(
        alu_add_inc[0]), .F(n174), .Z(alu_out[0]) );
  HS65_GS_OAI212X5 U243 ( .A(n11), .B(n41), .C(n19), .D(n40), .E(n268), .Z(
        n267) );
  HS65_GS_AOI212X4 U244 ( .A(inst_alu[10]), .B(op_src[1]), .C(op_dst[0]), .D(
        n269), .E(n270), .Z(n268) );
  HS65_GS_IVX9 U245 ( .A(inst_alu[6]), .Z(n45) );
  HS65_GS_OA31X9 U246 ( .A(n271), .B(inst_alu[4]), .C(inst_alu[10]), .D(n46), 
        .Z(n191) );
  HS65_GS_NAND3X5 U247 ( .A(n41), .B(n40), .C(n45), .Z(n271) );
  HS65_GS_NOR2X6 U248 ( .A(n174), .B(inst_alu[7]), .Z(n184) );
  HS65_GS_OR3X9 U249 ( .A(inst_so[7]), .B(inst_alu[3]), .C(dbg_halt_st), .Z(
        n174) );
  HS65_GS_IVX9 U250 ( .A(inst_so[1]), .Z(n41) );
  HS65_GS_IVX9 U251 ( .A(inst_alu[10]), .Z(n43) );
  HS65_GS_IVX9 U252 ( .A(inst_alu[5]), .Z(n46) );
  HS65_GS_IVX9 U253 ( .A(inst_alu[4]), .Z(n47) );
  HS65_GS_IVX9 U254 ( .A(inst_so[3]), .Z(n40) );
  HS65_GS_CB4I1X9 U255 ( .A(inst_alu[2]), .B(status[0]), .C(inst_alu[1]), .D(
        exec_cycle), .Z(alu_inc) );
  HS65_GS_NOR2X6 U256 ( .A(n169), .B(inst_alu[10]), .Z(n161) );
  HS65_GS_OR3X9 U257 ( .A(inst_alu[8]), .B(inst_alu[10]), .C(n9), .Z(n164) );
  HS65_GS_AND2X4 U258 ( .A(inst_alu[9]), .B(exec_cycle), .Z(alu_stat_wr[2]) );
  HS65_GS_NAND2AX7 U259 ( .A(inst_alu[8]), .B(n45), .Z(n169) );
  HS65_GS_IVX9 U260 ( .A(n5), .Z(alu_stat_wr[1]) );
  HS65_GS_IVX9 U261 ( .A(n5), .Z(alu_stat_wr[3]) );
  HS65_GS_IVX9 U262 ( .A(n5), .Z(alu_stat_wr[0]) );
  HS65_GSS_XOR2X3 U263 ( .A(alu_add_16_), .B(add_180_carry[16]), .Z(
        alu_add_inc[16]) );
  HS65_GSS_XOR2X3 U264 ( .A(N54), .B(add_102_C188_aco_carry[4]), .Z(
        alu_dadd3[4]) );
  HS65_GS_AND2X4 U265 ( .A(add_180_carry[15]), .B(alu_out_add[15]), .Z(
        add_180_carry[16]) );
  HS65_GSS_XOR2X3 U266 ( .A(add_180_carry[15]), .B(alu_out_add[15]), .Z(
        alu_add_inc[15]) );
  HS65_GS_AND2X4 U267 ( .A(add_180_carry[14]), .B(alu_out_add[14]), .Z(
        add_180_carry[15]) );
  HS65_GSS_XOR2X3 U268 ( .A(add_180_carry[14]), .B(alu_out_add[14]), .Z(
        alu_add_inc[14]) );
  HS65_GS_AND2X4 U269 ( .A(add_180_carry[13]), .B(alu_out_add[13]), .Z(
        add_180_carry[14]) );
  HS65_GSS_XOR2X3 U270 ( .A(add_180_carry[13]), .B(alu_out_add[13]), .Z(
        alu_add_inc[13]) );
  HS65_GS_AND2X4 U271 ( .A(add_180_carry[12]), .B(alu_out_add[12]), .Z(
        add_180_carry[13]) );
  HS65_GSS_XOR2X3 U272 ( .A(add_180_carry[12]), .B(alu_out_add[12]), .Z(
        alu_add_inc[12]) );
  HS65_GS_AND2X4 U273 ( .A(add_180_carry[11]), .B(alu_out_add[11]), .Z(
        add_180_carry[12]) );
  HS65_GSS_XOR2X3 U274 ( .A(add_180_carry[11]), .B(alu_out_add[11]), .Z(
        alu_add_inc[11]) );
  HS65_GS_AND2X4 U275 ( .A(add_180_carry[10]), .B(alu_out_add[10]), .Z(
        add_180_carry[11]) );
  HS65_GSS_XOR2X3 U276 ( .A(add_180_carry[10]), .B(alu_out_add[10]), .Z(
        alu_add_inc[10]) );
  HS65_GS_AND2X4 U277 ( .A(add_180_carry[9]), .B(alu_out_add[9]), .Z(
        add_180_carry[10]) );
  HS65_GSS_XOR2X3 U278 ( .A(add_180_carry[9]), .B(alu_out_add[9]), .Z(
        alu_add_inc[9]) );
  HS65_GS_AND2X4 U279 ( .A(add_180_carry[8]), .B(alu_out_add[8]), .Z(
        add_180_carry[9]) );
  HS65_GSS_XOR2X3 U280 ( .A(add_180_carry[8]), .B(alu_out_add[8]), .Z(
        alu_add_inc[8]) );
  HS65_GS_AND2X4 U281 ( .A(add_180_carry[7]), .B(alu_out_add[7]), .Z(
        add_180_carry[8]) );
  HS65_GSS_XOR2X3 U282 ( .A(add_180_carry[7]), .B(alu_out_add[7]), .Z(
        alu_add_inc[7]) );
  HS65_GS_AND2X4 U283 ( .A(add_180_carry[6]), .B(alu_out_add[6]), .Z(
        add_180_carry[7]) );
  HS65_GSS_XOR2X3 U284 ( .A(add_180_carry[6]), .B(alu_out_add[6]), .Z(
        alu_add_inc[6]) );
  HS65_GS_AND2X4 U285 ( .A(add_180_carry[5]), .B(alu_out_add[5]), .Z(
        add_180_carry[6]) );
  HS65_GSS_XOR2X3 U286 ( .A(add_180_carry[5]), .B(alu_out_add[5]), .Z(
        alu_add_inc[5]) );
  HS65_GS_AND2X4 U287 ( .A(add_180_carry[4]), .B(alu_out_add[4]), .Z(
        add_180_carry[5]) );
  HS65_GSS_XOR2X3 U288 ( .A(add_180_carry[4]), .B(alu_out_add[4]), .Z(
        alu_add_inc[4]) );
  HS65_GS_AND2X4 U289 ( .A(add_180_carry[3]), .B(alu_out_add[3]), .Z(
        add_180_carry[4]) );
  HS65_GSS_XOR2X3 U290 ( .A(add_180_carry[3]), .B(alu_out_add[3]), .Z(
        alu_add_inc[3]) );
  HS65_GS_AND2X4 U291 ( .A(add_180_carry[2]), .B(alu_out_add[2]), .Z(
        add_180_carry[3]) );
  HS65_GSS_XOR2X3 U292 ( .A(add_180_carry[2]), .B(alu_out_add[2]), .Z(
        alu_add_inc[2]) );
  HS65_GS_AND2X4 U293 ( .A(add_180_carry[1]), .B(alu_out_add[1]), .Z(
        add_180_carry[2]) );
  HS65_GSS_XOR2X3 U294 ( .A(add_180_carry[1]), .B(alu_out_add[1]), .Z(
        alu_add_inc[1]) );
  HS65_GS_AND2X4 U295 ( .A(alu_out_add[0]), .B(alu_inc), .Z(add_180_carry[1])
         );
  HS65_GSS_XOR2X3 U296 ( .A(alu_out_add[0]), .B(alu_inc), .Z(alu_add_inc[0])
         );
  HS65_GS_AND2X4 U297 ( .A(add_102_C188_aco_carry[3]), .B(N53), .Z(
        add_102_C188_aco_carry[4]) );
  HS65_GSS_XOR2X3 U298 ( .A(add_102_C188_aco_carry[3]), .B(N53), .Z(
        alu_dadd3[3]) );
  HS65_GS_AND2X4 U299 ( .A(N51), .B(n2), .Z(add_102_C188_aco_carry[2]) );
  HS65_GSS_XOR2X3 U300 ( .A(N51), .B(n2), .Z(alu_dadd3[1]) );
  HS65_GSS_XOR2X3 U301 ( .A(N41), .B(add_102_C187_aco_carry[4]), .Z(
        alu_dadd2_4_) );
  HS65_GS_AND2X4 U302 ( .A(add_102_C187_aco_carry[3]), .B(N40), .Z(
        add_102_C187_aco_carry[4]) );
  HS65_GSS_XOR2X3 U303 ( .A(add_102_C187_aco_carry[3]), .B(N40), .Z(
        alu_dadd2[3]) );
  HS65_GS_AND2X4 U304 ( .A(N38), .B(n1), .Z(add_102_C187_aco_carry[2]) );
  HS65_GSS_XOR2X3 U305 ( .A(N38), .B(n1), .Z(alu_dadd2[1]) );
  HS65_GSS_XOR2X3 U306 ( .A(N28), .B(add_102_C186_aco_carry[4]), .Z(
        alu_dadd1_4_) );
  HS65_GS_AND2X4 U307 ( .A(add_102_C186_aco_carry[3]), .B(N27), .Z(
        add_102_C186_aco_carry[4]) );
  HS65_GSS_XOR2X3 U308 ( .A(add_102_C186_aco_carry[3]), .B(N27), .Z(
        alu_dadd1[3]) );
  HS65_GS_AND2X4 U309 ( .A(N25), .B(n4), .Z(add_102_C186_aco_carry[2]) );
  HS65_GSS_XOR2X3 U310 ( .A(N25), .B(n4), .Z(alu_dadd1[1]) );
  HS65_GSS_XOR2X3 U311 ( .A(N15), .B(add_102_C185_aco_carry[4]), .Z(
        alu_dadd0_4_) );
  HS65_GS_AND2X4 U312 ( .A(add_102_C185_aco_carry[3]), .B(N14), .Z(
        add_102_C185_aco_carry[4]) );
  HS65_GSS_XOR2X3 U313 ( .A(add_102_C185_aco_carry[3]), .B(N14), .Z(
        alu_dadd0[3]) );
  HS65_GS_AND2X4 U314 ( .A(N12), .B(n3), .Z(add_102_C185_aco_carry[2]) );
  HS65_GSS_XOR2X3 U315 ( .A(N12), .B(n3), .Z(alu_dadd0[1]) );
endmodule


module omsp_execution_unit ( cpuoff, dbg_reg_din, gie, mab, mb_en, mb_wr, 
        mdb_out, oscoff, pc_sw, pc_sw_wr, scg0, scg1, dbg_halt_st, 
        dbg_mem_dout, dbg_reg_wr, e_state, exec_done, inst_ad, inst_as, 
        inst_alu, inst_bw, inst_dest, inst_dext, inst_irq_rst, inst_jmp, 
        inst_mov, inst_sext, inst_so, inst_src, inst_type, mclk, mdb_in, pc, 
        pc_nxt, puc_rst, scan_enable );
  output [15:0] dbg_reg_din;
  output [15:0] mab;
  output [1:0] mb_wr;
  output [15:0] mdb_out;
  output [15:0] pc_sw;
  input [15:0] dbg_mem_dout;
  input [3:0] e_state;
  input [7:0] inst_ad;
  input [7:0] inst_as;
  input [11:0] inst_alu;
  input [15:0] inst_dest;
  input [15:0] inst_dext;
  input [7:0] inst_jmp;
  input [15:0] inst_sext;
  input [7:0] inst_so;
  input [15:0] inst_src;
  input [2:0] inst_type;
  input [15:0] mdb_in;
  input [15:0] pc;
  input [15:0] pc_nxt;
  input dbg_halt_st, dbg_reg_wr, exec_done, inst_bw, inst_irq_rst, inst_mov,
         mclk, puc_rst, scan_enable;
  output cpuoff, gie, mb_en, oscoff, pc_sw_wr, scg0, scg1;
  wire   n213, reg_dest_wr, reg_sp_wr, reg_sr_wr, reg_pc_call, reg_incr,
         mab_lsb, mdb_in_buf_en, mdb_in_buf_valid, n1, n2, n3, n4, n5, n6, n8,
         n9, n11, n12, n14, n18, n19, n20, n22, n23, n26, n27, n28, n29, n30,
         n32, n33, n35, n36, n38, n39, n41, n42, n44, n45, n47, n48, n51, n52,
         n54, n55, n57, n58, n60, n61, n63, n64, n66, n67, n68, n69, n70, n72,
         n73, n74, n75, n77, n79, n82, n83, n84, n85, n86, n87, n89, n90, n92,
         n93, n94, n95, n97, n98, n100, n101, n103, n104, n106, n107, n109,
         n110, n112, n113, n115, n116, n118, n120, n122, n124, n126, n128,
         n130, n133, n134, n138, n139, n140, n143, n144, n145, n147, n149,
         n150, n151, n152, n157, n158, n159, n161, n162, n163, n164, n165,
         n166, n167, n168, n169, n170, n171, n172, n173, n174, n175, n176,
         n177, n181, n182, n183, n184, n185, n186, n187, n188, n189, n190,
         n191, n192, n193, n194, n195, n196, n197, n198, n199, n25, n34, n37,
         n40, n43, n46, n49, n50, n53, n56, n59, n62, n65, n71, n76, n78, n80,
         n81, n88, n91, n96, n99, n102, n105, n108, n111, n114, n117, n119,
         n121, n123, n125, n127, n129, n131, n132, n135, n136, n137, n141,
         n142, n146, n148, n153, n154, n155, n156, n160, n178, n179, n180,
         n200, n201, n202, n203, n204, n205, n206, n207, n208, n209, n210,
         SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2,
         SYNOPSYS_UNCONNECTED_3, SYNOPSYS_UNCONNECTED_4,
         SYNOPSYS_UNCONNECTED_5, SYNOPSYS_UNCONNECTED_6,
         SYNOPSYS_UNCONNECTED_7, SYNOPSYS_UNCONNECTED_8,
         SYNOPSYS_UNCONNECTED_9, SYNOPSYS_UNCONNECTED_10,
         SYNOPSYS_UNCONNECTED_11, SYNOPSYS_UNCONNECTED_12,
         SYNOPSYS_UNCONNECTED_13, SYNOPSYS_UNCONNECTED_14,
         SYNOPSYS_UNCONNECTED_15, SYNOPSYS_UNCONNECTED_16,
         SYNOPSYS_UNCONNECTED_17, SYNOPSYS_UNCONNECTED_18,
         SYNOPSYS_UNCONNECTED_19, SYNOPSYS_UNCONNECTED_20,
         SYNOPSYS_UNCONNECTED_21, SYNOPSYS_UNCONNECTED_22,
         SYNOPSYS_UNCONNECTED_23, SYNOPSYS_UNCONNECTED_24,
         SYNOPSYS_UNCONNECTED_25, SYNOPSYS_UNCONNECTED_26,
         SYNOPSYS_UNCONNECTED_27, SYNOPSYS_UNCONNECTED_28,
         SYNOPSYS_UNCONNECTED_29, SYNOPSYS_UNCONNECTED_30,
         SYNOPSYS_UNCONNECTED_31, SYNOPSYS_UNCONNECTED_32,
         SYNOPSYS_UNCONNECTED_33, SYNOPSYS_UNCONNECTED_34,
         SYNOPSYS_UNCONNECTED_35, SYNOPSYS_UNCONNECTED_36,
         SYNOPSYS_UNCONNECTED_37, SYNOPSYS_UNCONNECTED_38,
         SYNOPSYS_UNCONNECTED_39, SYNOPSYS_UNCONNECTED_40,
         SYNOPSYS_UNCONNECTED_41, SYNOPSYS_UNCONNECTED_42,
         SYNOPSYS_UNCONNECTED_43, SYNOPSYS_UNCONNECTED_44,
         SYNOPSYS_UNCONNECTED_45, SYNOPSYS_UNCONNECTED_46,
         SYNOPSYS_UNCONNECTED_47, SYNOPSYS_UNCONNECTED_48,
         SYNOPSYS_UNCONNECTED_49, SYNOPSYS_UNCONNECTED_50,
         SYNOPSYS_UNCONNECTED_51, SYNOPSYS_UNCONNECTED_52,
         SYNOPSYS_UNCONNECTED_53, SYNOPSYS_UNCONNECTED_54,
         SYNOPSYS_UNCONNECTED_55, SYNOPSYS_UNCONNECTED_56,
         SYNOPSYS_UNCONNECTED_57, SYNOPSYS_UNCONNECTED_58,
         SYNOPSYS_UNCONNECTED_59, SYNOPSYS_UNCONNECTED_60,
         SYNOPSYS_UNCONNECTED_61, SYNOPSYS_UNCONNECTED_62,
         SYNOPSYS_UNCONNECTED_63, SYNOPSYS_UNCONNECTED_64,
         SYNOPSYS_UNCONNECTED_65, SYNOPSYS_UNCONNECTED_66,
         SYNOPSYS_UNCONNECTED_67, SYNOPSYS_UNCONNECTED_68,
         SYNOPSYS_UNCONNECTED_69, SYNOPSYS_UNCONNECTED_70,
         SYNOPSYS_UNCONNECTED_71, SYNOPSYS_UNCONNECTED_72,
         SYNOPSYS_UNCONNECTED_73, SYNOPSYS_UNCONNECTED_74,
         SYNOPSYS_UNCONNECTED_75, SYNOPSYS_UNCONNECTED_76,
         SYNOPSYS_UNCONNECTED_77, SYNOPSYS_UNCONNECTED_78,
         SYNOPSYS_UNCONNECTED_79, SYNOPSYS_UNCONNECTED_80,
         SYNOPSYS_UNCONNECTED_81, SYNOPSYS_UNCONNECTED_82,
         SYNOPSYS_UNCONNECTED_83, SYNOPSYS_UNCONNECTED_84,
         SYNOPSYS_UNCONNECTED_85, SYNOPSYS_UNCONNECTED_86,
         SYNOPSYS_UNCONNECTED_87, SYNOPSYS_UNCONNECTED_88,
         SYNOPSYS_UNCONNECTED_89, SYNOPSYS_UNCONNECTED_90,
         SYNOPSYS_UNCONNECTED_91, SYNOPSYS_UNCONNECTED_92,
         SYNOPSYS_UNCONNECTED_93, SYNOPSYS_UNCONNECTED_94,
         SYNOPSYS_UNCONNECTED_95, SYNOPSYS_UNCONNECTED_96,
         SYNOPSYS_UNCONNECTED_97, SYNOPSYS_UNCONNECTED_98,
         SYNOPSYS_UNCONNECTED_99, SYNOPSYS_UNCONNECTED_100,
         SYNOPSYS_UNCONNECTED_101, SYNOPSYS_UNCONNECTED_102,
         SYNOPSYS_UNCONNECTED_103, SYNOPSYS_UNCONNECTED_104,
         SYNOPSYS_UNCONNECTED_105, SYNOPSYS_UNCONNECTED_106,
         SYNOPSYS_UNCONNECTED_107, SYNOPSYS_UNCONNECTED_108,
         SYNOPSYS_UNCONNECTED_109, SYNOPSYS_UNCONNECTED_110,
         SYNOPSYS_UNCONNECTED_111, SYNOPSYS_UNCONNECTED_112,
         SYNOPSYS_UNCONNECTED_113, SYNOPSYS_UNCONNECTED_114,
         SYNOPSYS_UNCONNECTED_115, SYNOPSYS_UNCONNECTED_116,
         SYNOPSYS_UNCONNECTED_117, SYNOPSYS_UNCONNECTED_118,
         SYNOPSYS_UNCONNECTED_119, SYNOPSYS_UNCONNECTED_120,
         SYNOPSYS_UNCONNECTED_121, SYNOPSYS_UNCONNECTED_122,
         SYNOPSYS_UNCONNECTED_123, SYNOPSYS_UNCONNECTED_124,
         SYNOPSYS_UNCONNECTED_125, SYNOPSYS_UNCONNECTED_126,
         SYNOPSYS_UNCONNECTED_127, SYNOPSYS_UNCONNECTED_128,
         SYNOPSYS_UNCONNECTED_129, SYNOPSYS_UNCONNECTED_130,
         SYNOPSYS_UNCONNECTED_131, SYNOPSYS_UNCONNECTED_132,
         SYNOPSYS_UNCONNECTED_133, SYNOPSYS_UNCONNECTED_134,
         SYNOPSYS_UNCONNECTED_135, SYNOPSYS_UNCONNECTED_136,
         SYNOPSYS_UNCONNECTED_137, SYNOPSYS_UNCONNECTED_138,
         SYNOPSYS_UNCONNECTED_139, SYNOPSYS_UNCONNECTED_140,
         SYNOPSYS_UNCONNECTED_141, SYNOPSYS_UNCONNECTED_142,
         SYNOPSYS_UNCONNECTED_143, SYNOPSYS_UNCONNECTED_144,
         SYNOPSYS_UNCONNECTED_145, SYNOPSYS_UNCONNECTED_146,
         SYNOPSYS_UNCONNECTED_147, SYNOPSYS_UNCONNECTED_148,
         SYNOPSYS_UNCONNECTED_149, SYNOPSYS_UNCONNECTED_150,
         SYNOPSYS_UNCONNECTED_151, SYNOPSYS_UNCONNECTED_152,
         SYNOPSYS_UNCONNECTED_153, SYNOPSYS_UNCONNECTED_154,
         SYNOPSYS_UNCONNECTED_155, SYNOPSYS_UNCONNECTED_156,
         SYNOPSYS_UNCONNECTED_157, SYNOPSYS_UNCONNECTED_158,
         SYNOPSYS_UNCONNECTED_159, SYNOPSYS_UNCONNECTED_160,
         SYNOPSYS_UNCONNECTED_161, SYNOPSYS_UNCONNECTED_162,
         SYNOPSYS_UNCONNECTED_163, SYNOPSYS_UNCONNECTED_164,
         SYNOPSYS_UNCONNECTED_165, SYNOPSYS_UNCONNECTED_166,
         SYNOPSYS_UNCONNECTED_167, SYNOPSYS_UNCONNECTED_168,
         SYNOPSYS_UNCONNECTED_169, SYNOPSYS_UNCONNECTED_170,
         SYNOPSYS_UNCONNECTED_171, SYNOPSYS_UNCONNECTED_172,
         SYNOPSYS_UNCONNECTED_173, SYNOPSYS_UNCONNECTED_174,
         SYNOPSYS_UNCONNECTED_175, SYNOPSYS_UNCONNECTED_176,
         SYNOPSYS_UNCONNECTED_177, SYNOPSYS_UNCONNECTED_178,
         SYNOPSYS_UNCONNECTED_179, SYNOPSYS_UNCONNECTED_180,
         SYNOPSYS_UNCONNECTED_181, SYNOPSYS_UNCONNECTED_182,
         SYNOPSYS_UNCONNECTED_183, SYNOPSYS_UNCONNECTED_184,
         SYNOPSYS_UNCONNECTED_185, SYNOPSYS_UNCONNECTED_186,
         SYNOPSYS_UNCONNECTED_187, SYNOPSYS_UNCONNECTED_188,
         SYNOPSYS_UNCONNECTED_189, SYNOPSYS_UNCONNECTED_190,
         SYNOPSYS_UNCONNECTED_191, SYNOPSYS_UNCONNECTED_192,
         SYNOPSYS_UNCONNECTED_193, SYNOPSYS_UNCONNECTED_194,
         SYNOPSYS_UNCONNECTED_195, SYNOPSYS_UNCONNECTED_196,
         SYNOPSYS_UNCONNECTED_197, SYNOPSYS_UNCONNECTED_198,
         SYNOPSYS_UNCONNECTED_199, SYNOPSYS_UNCONNECTED_200,
         SYNOPSYS_UNCONNECTED_201, SYNOPSYS_UNCONNECTED_202,
         SYNOPSYS_UNCONNECTED_203, SYNOPSYS_UNCONNECTED_204,
         SYNOPSYS_UNCONNECTED_205, SYNOPSYS_UNCONNECTED_206,
         SYNOPSYS_UNCONNECTED_207, SYNOPSYS_UNCONNECTED_208,
         SYNOPSYS_UNCONNECTED_209, SYNOPSYS_UNCONNECTED_210,
         SYNOPSYS_UNCONNECTED_211, SYNOPSYS_UNCONNECTED_212,
         SYNOPSYS_UNCONNECTED_213, SYNOPSYS_UNCONNECTED_214,
         SYNOPSYS_UNCONNECTED_215, SYNOPSYS_UNCONNECTED_216,
         SYNOPSYS_UNCONNECTED_217, SYNOPSYS_UNCONNECTED_218,
         SYNOPSYS_UNCONNECTED_219, SYNOPSYS_UNCONNECTED_220,
         SYNOPSYS_UNCONNECTED_221, SYNOPSYS_UNCONNECTED_222,
         SYNOPSYS_UNCONNECTED_223, SYNOPSYS_UNCONNECTED_224,
         SYNOPSYS_UNCONNECTED_225, SYNOPSYS_UNCONNECTED_226,
         SYNOPSYS_UNCONNECTED_227, SYNOPSYS_UNCONNECTED_228,
         SYNOPSYS_UNCONNECTED_229, SYNOPSYS_UNCONNECTED_230,
         SYNOPSYS_UNCONNECTED_231, SYNOPSYS_UNCONNECTED_232,
         SYNOPSYS_UNCONNECTED_233, SYNOPSYS_UNCONNECTED_234,
         SYNOPSYS_UNCONNECTED_235, SYNOPSYS_UNCONNECTED_236,
         SYNOPSYS_UNCONNECTED_237, SYNOPSYS_UNCONNECTED_238,
         SYNOPSYS_UNCONNECTED_239, SYNOPSYS_UNCONNECTED_240,
         SYNOPSYS_UNCONNECTED_241, SYNOPSYS_UNCONNECTED_242,
         SYNOPSYS_UNCONNECTED_243, SYNOPSYS_UNCONNECTED_244,
         SYNOPSYS_UNCONNECTED_245, SYNOPSYS_UNCONNECTED_246,
         SYNOPSYS_UNCONNECTED_247, SYNOPSYS_UNCONNECTED_248,
         SYNOPSYS_UNCONNECTED_249, SYNOPSYS_UNCONNECTED_250,
         SYNOPSYS_UNCONNECTED_251, SYNOPSYS_UNCONNECTED_252,
         SYNOPSYS_UNCONNECTED_253, SYNOPSYS_UNCONNECTED_254,
         SYNOPSYS_UNCONNECTED_255, SYNOPSYS_UNCONNECTED_256;
  wire   [15:0] reg_src;
  wire   [3:0] status;
  wire   [3:0] alu_stat;
  wire   [3:0] alu_stat_wr;
  wire   [15:0] alu_out;
  wire   [15:0] op_src;
  wire   [15:0] op_dst;
  wire   [15:8] mdb_out_nxt;
  wire   [15:0] mdb_in_buf;

  omsp_register_file register_file_0 ( .gie(gie), .pc_sw(pc_sw), .pc_sw_wr(
        n213), .reg_dest(dbg_reg_din), .reg_src(reg_src), .status(status), 
        .OBSERVE_r0({SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2, 
        SYNOPSYS_UNCONNECTED_3, SYNOPSYS_UNCONNECTED_4, SYNOPSYS_UNCONNECTED_5, 
        SYNOPSYS_UNCONNECTED_6, SYNOPSYS_UNCONNECTED_7, SYNOPSYS_UNCONNECTED_8, 
        SYNOPSYS_UNCONNECTED_9, SYNOPSYS_UNCONNECTED_10, 
        SYNOPSYS_UNCONNECTED_11, SYNOPSYS_UNCONNECTED_12, 
        SYNOPSYS_UNCONNECTED_13, SYNOPSYS_UNCONNECTED_14, 
        SYNOPSYS_UNCONNECTED_15, SYNOPSYS_UNCONNECTED_16}), .OBSERVE_r1({
        SYNOPSYS_UNCONNECTED_17, SYNOPSYS_UNCONNECTED_18, 
        SYNOPSYS_UNCONNECTED_19, SYNOPSYS_UNCONNECTED_20, 
        SYNOPSYS_UNCONNECTED_21, SYNOPSYS_UNCONNECTED_22, 
        SYNOPSYS_UNCONNECTED_23, SYNOPSYS_UNCONNECTED_24, 
        SYNOPSYS_UNCONNECTED_25, SYNOPSYS_UNCONNECTED_26, 
        SYNOPSYS_UNCONNECTED_27, SYNOPSYS_UNCONNECTED_28, 
        SYNOPSYS_UNCONNECTED_29, SYNOPSYS_UNCONNECTED_30, 
        SYNOPSYS_UNCONNECTED_31, SYNOPSYS_UNCONNECTED_32}), .OBSERVE_r2({
        SYNOPSYS_UNCONNECTED_33, SYNOPSYS_UNCONNECTED_34, 
        SYNOPSYS_UNCONNECTED_35, SYNOPSYS_UNCONNECTED_36, 
        SYNOPSYS_UNCONNECTED_37, SYNOPSYS_UNCONNECTED_38, 
        SYNOPSYS_UNCONNECTED_39, SYNOPSYS_UNCONNECTED_40, 
        SYNOPSYS_UNCONNECTED_41, SYNOPSYS_UNCONNECTED_42, 
        SYNOPSYS_UNCONNECTED_43, SYNOPSYS_UNCONNECTED_44, 
        SYNOPSYS_UNCONNECTED_45, SYNOPSYS_UNCONNECTED_46, 
        SYNOPSYS_UNCONNECTED_47, SYNOPSYS_UNCONNECTED_48}), .OBSERVE_r3({
        SYNOPSYS_UNCONNECTED_49, SYNOPSYS_UNCONNECTED_50, 
        SYNOPSYS_UNCONNECTED_51, SYNOPSYS_UNCONNECTED_52, 
        SYNOPSYS_UNCONNECTED_53, SYNOPSYS_UNCONNECTED_54, 
        SYNOPSYS_UNCONNECTED_55, SYNOPSYS_UNCONNECTED_56, 
        SYNOPSYS_UNCONNECTED_57, SYNOPSYS_UNCONNECTED_58, 
        SYNOPSYS_UNCONNECTED_59, SYNOPSYS_UNCONNECTED_60, 
        SYNOPSYS_UNCONNECTED_61, SYNOPSYS_UNCONNECTED_62, 
        SYNOPSYS_UNCONNECTED_63, SYNOPSYS_UNCONNECTED_64}), .OBSERVE_r4({
        SYNOPSYS_UNCONNECTED_65, SYNOPSYS_UNCONNECTED_66, 
        SYNOPSYS_UNCONNECTED_67, SYNOPSYS_UNCONNECTED_68, 
        SYNOPSYS_UNCONNECTED_69, SYNOPSYS_UNCONNECTED_70, 
        SYNOPSYS_UNCONNECTED_71, SYNOPSYS_UNCONNECTED_72, 
        SYNOPSYS_UNCONNECTED_73, SYNOPSYS_UNCONNECTED_74, 
        SYNOPSYS_UNCONNECTED_75, SYNOPSYS_UNCONNECTED_76, 
        SYNOPSYS_UNCONNECTED_77, SYNOPSYS_UNCONNECTED_78, 
        SYNOPSYS_UNCONNECTED_79, SYNOPSYS_UNCONNECTED_80}), .OBSERVE_r5({
        SYNOPSYS_UNCONNECTED_81, SYNOPSYS_UNCONNECTED_82, 
        SYNOPSYS_UNCONNECTED_83, SYNOPSYS_UNCONNECTED_84, 
        SYNOPSYS_UNCONNECTED_85, SYNOPSYS_UNCONNECTED_86, 
        SYNOPSYS_UNCONNECTED_87, SYNOPSYS_UNCONNECTED_88, 
        SYNOPSYS_UNCONNECTED_89, SYNOPSYS_UNCONNECTED_90, 
        SYNOPSYS_UNCONNECTED_91, SYNOPSYS_UNCONNECTED_92, 
        SYNOPSYS_UNCONNECTED_93, SYNOPSYS_UNCONNECTED_94, 
        SYNOPSYS_UNCONNECTED_95, SYNOPSYS_UNCONNECTED_96}), .OBSERVE_r6({
        SYNOPSYS_UNCONNECTED_97, SYNOPSYS_UNCONNECTED_98, 
        SYNOPSYS_UNCONNECTED_99, SYNOPSYS_UNCONNECTED_100, 
        SYNOPSYS_UNCONNECTED_101, SYNOPSYS_UNCONNECTED_102, 
        SYNOPSYS_UNCONNECTED_103, SYNOPSYS_UNCONNECTED_104, 
        SYNOPSYS_UNCONNECTED_105, SYNOPSYS_UNCONNECTED_106, 
        SYNOPSYS_UNCONNECTED_107, SYNOPSYS_UNCONNECTED_108, 
        SYNOPSYS_UNCONNECTED_109, SYNOPSYS_UNCONNECTED_110, 
        SYNOPSYS_UNCONNECTED_111, SYNOPSYS_UNCONNECTED_112}), .OBSERVE_r7({
        SYNOPSYS_UNCONNECTED_113, SYNOPSYS_UNCONNECTED_114, 
        SYNOPSYS_UNCONNECTED_115, SYNOPSYS_UNCONNECTED_116, 
        SYNOPSYS_UNCONNECTED_117, SYNOPSYS_UNCONNECTED_118, 
        SYNOPSYS_UNCONNECTED_119, SYNOPSYS_UNCONNECTED_120, 
        SYNOPSYS_UNCONNECTED_121, SYNOPSYS_UNCONNECTED_122, 
        SYNOPSYS_UNCONNECTED_123, SYNOPSYS_UNCONNECTED_124, 
        SYNOPSYS_UNCONNECTED_125, SYNOPSYS_UNCONNECTED_126, 
        SYNOPSYS_UNCONNECTED_127, SYNOPSYS_UNCONNECTED_128}), .OBSERVE_r8({
        SYNOPSYS_UNCONNECTED_129, SYNOPSYS_UNCONNECTED_130, 
        SYNOPSYS_UNCONNECTED_131, SYNOPSYS_UNCONNECTED_132, 
        SYNOPSYS_UNCONNECTED_133, SYNOPSYS_UNCONNECTED_134, 
        SYNOPSYS_UNCONNECTED_135, SYNOPSYS_UNCONNECTED_136, 
        SYNOPSYS_UNCONNECTED_137, SYNOPSYS_UNCONNECTED_138, 
        SYNOPSYS_UNCONNECTED_139, SYNOPSYS_UNCONNECTED_140, 
        SYNOPSYS_UNCONNECTED_141, SYNOPSYS_UNCONNECTED_142, 
        SYNOPSYS_UNCONNECTED_143, SYNOPSYS_UNCONNECTED_144}), .OBSERVE_r9({
        SYNOPSYS_UNCONNECTED_145, SYNOPSYS_UNCONNECTED_146, 
        SYNOPSYS_UNCONNECTED_147, SYNOPSYS_UNCONNECTED_148, 
        SYNOPSYS_UNCONNECTED_149, SYNOPSYS_UNCONNECTED_150, 
        SYNOPSYS_UNCONNECTED_151, SYNOPSYS_UNCONNECTED_152, 
        SYNOPSYS_UNCONNECTED_153, SYNOPSYS_UNCONNECTED_154, 
        SYNOPSYS_UNCONNECTED_155, SYNOPSYS_UNCONNECTED_156, 
        SYNOPSYS_UNCONNECTED_157, SYNOPSYS_UNCONNECTED_158, 
        SYNOPSYS_UNCONNECTED_159, SYNOPSYS_UNCONNECTED_160}), .OBSERVE_r10({
        SYNOPSYS_UNCONNECTED_161, SYNOPSYS_UNCONNECTED_162, 
        SYNOPSYS_UNCONNECTED_163, SYNOPSYS_UNCONNECTED_164, 
        SYNOPSYS_UNCONNECTED_165, SYNOPSYS_UNCONNECTED_166, 
        SYNOPSYS_UNCONNECTED_167, SYNOPSYS_UNCONNECTED_168, 
        SYNOPSYS_UNCONNECTED_169, SYNOPSYS_UNCONNECTED_170, 
        SYNOPSYS_UNCONNECTED_171, SYNOPSYS_UNCONNECTED_172, 
        SYNOPSYS_UNCONNECTED_173, SYNOPSYS_UNCONNECTED_174, 
        SYNOPSYS_UNCONNECTED_175, SYNOPSYS_UNCONNECTED_176}), .OBSERVE_r11({
        SYNOPSYS_UNCONNECTED_177, SYNOPSYS_UNCONNECTED_178, 
        SYNOPSYS_UNCONNECTED_179, SYNOPSYS_UNCONNECTED_180, 
        SYNOPSYS_UNCONNECTED_181, SYNOPSYS_UNCONNECTED_182, 
        SYNOPSYS_UNCONNECTED_183, SYNOPSYS_UNCONNECTED_184, 
        SYNOPSYS_UNCONNECTED_185, SYNOPSYS_UNCONNECTED_186, 
        SYNOPSYS_UNCONNECTED_187, SYNOPSYS_UNCONNECTED_188, 
        SYNOPSYS_UNCONNECTED_189, SYNOPSYS_UNCONNECTED_190, 
        SYNOPSYS_UNCONNECTED_191, SYNOPSYS_UNCONNECTED_192}), .OBSERVE_r12({
        SYNOPSYS_UNCONNECTED_193, SYNOPSYS_UNCONNECTED_194, 
        SYNOPSYS_UNCONNECTED_195, SYNOPSYS_UNCONNECTED_196, 
        SYNOPSYS_UNCONNECTED_197, SYNOPSYS_UNCONNECTED_198, 
        SYNOPSYS_UNCONNECTED_199, SYNOPSYS_UNCONNECTED_200, 
        SYNOPSYS_UNCONNECTED_201, SYNOPSYS_UNCONNECTED_202, 
        SYNOPSYS_UNCONNECTED_203, SYNOPSYS_UNCONNECTED_204, 
        SYNOPSYS_UNCONNECTED_205, SYNOPSYS_UNCONNECTED_206, 
        SYNOPSYS_UNCONNECTED_207, SYNOPSYS_UNCONNECTED_208}), .OBSERVE_r13({
        SYNOPSYS_UNCONNECTED_209, SYNOPSYS_UNCONNECTED_210, 
        SYNOPSYS_UNCONNECTED_211, SYNOPSYS_UNCONNECTED_212, 
        SYNOPSYS_UNCONNECTED_213, SYNOPSYS_UNCONNECTED_214, 
        SYNOPSYS_UNCONNECTED_215, SYNOPSYS_UNCONNECTED_216, 
        SYNOPSYS_UNCONNECTED_217, SYNOPSYS_UNCONNECTED_218, 
        SYNOPSYS_UNCONNECTED_219, SYNOPSYS_UNCONNECTED_220, 
        SYNOPSYS_UNCONNECTED_221, SYNOPSYS_UNCONNECTED_222, 
        SYNOPSYS_UNCONNECTED_223, SYNOPSYS_UNCONNECTED_224}), .OBSERVE_r14({
        SYNOPSYS_UNCONNECTED_225, SYNOPSYS_UNCONNECTED_226, 
        SYNOPSYS_UNCONNECTED_227, SYNOPSYS_UNCONNECTED_228, 
        SYNOPSYS_UNCONNECTED_229, SYNOPSYS_UNCONNECTED_230, 
        SYNOPSYS_UNCONNECTED_231, SYNOPSYS_UNCONNECTED_232, 
        SYNOPSYS_UNCONNECTED_233, SYNOPSYS_UNCONNECTED_234, 
        SYNOPSYS_UNCONNECTED_235, SYNOPSYS_UNCONNECTED_236, 
        SYNOPSYS_UNCONNECTED_237, SYNOPSYS_UNCONNECTED_238, 
        SYNOPSYS_UNCONNECTED_239, SYNOPSYS_UNCONNECTED_240}), .OBSERVE_r15({
        SYNOPSYS_UNCONNECTED_241, SYNOPSYS_UNCONNECTED_242, 
        SYNOPSYS_UNCONNECTED_243, SYNOPSYS_UNCONNECTED_244, 
        SYNOPSYS_UNCONNECTED_245, SYNOPSYS_UNCONNECTED_246, 
        SYNOPSYS_UNCONNECTED_247, SYNOPSYS_UNCONNECTED_248, 
        SYNOPSYS_UNCONNECTED_249, SYNOPSYS_UNCONNECTED_250, 
        SYNOPSYS_UNCONNECTED_251, SYNOPSYS_UNCONNECTED_252, 
        SYNOPSYS_UNCONNECTED_253, SYNOPSYS_UNCONNECTED_254, 
        SYNOPSYS_UNCONNECTED_255, SYNOPSYS_UNCONNECTED_256}), .alu_stat(
        alu_stat), .alu_stat_wr(alu_stat_wr), .inst_bw(n40), .inst_dest(
        inst_dest), .inst_src(inst_src), .mclk(mclk), .pc(pc), .puc_rst(
        puc_rst), .reg_dest_val(alu_out), .reg_dest_wr(reg_dest_wr), 
        .reg_pc_call(reg_pc_call), .reg_sp_val(mab), .reg_sp_wr(reg_sp_wr), 
        .reg_sr_wr(reg_sr_wr), .reg_sr_clr(n181), .reg_incr(reg_incr), 
        .scan_enable(scan_enable) );
  omsp_alu alu_0 ( .alu_out(alu_out), .alu_out_add(mab), .alu_stat(alu_stat), 
        .alu_stat_wr(alu_stat_wr), .dbg_halt_st(dbg_halt_st), .exec_cycle(n155), .inst_alu(inst_alu), .inst_bw(n40), .inst_jmp(inst_jmp), .inst_so(inst_so), 
        .op_dst(op_dst), .op_src(op_src), .status(status) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_8_ ( .D(n170), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[8]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_15_ ( .D(n177), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[15]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_14_ ( .D(n176), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[14]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_13_ ( .D(n175), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[13]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_12_ ( .D(n174), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[12]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_11_ ( .D(n173), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[11]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_10_ ( .D(n172), .CP(mclk), .RN(n43), .Q(
        mdb_in_buf[10]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_9_ ( .D(n171), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[9]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_0_ ( .D(n169), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[0]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_1_ ( .D(n162), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[1]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_7_ ( .D(n168), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[7]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_6_ ( .D(n167), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[6]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_5_ ( .D(n166), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[5]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_4_ ( .D(n165), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[4]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_3_ ( .D(n164), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[3]) );
  HS65_GS_DFPRQX4 mdb_in_buf_reg_2_ ( .D(n163), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf[2]) );
  HS65_GS_DFPRQX4 mab_lsb_reg ( .D(n198), .CP(mclk), .RN(n25), .Q(mab_lsb) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_0_ ( .D(n197), .CP(mclk), .RN(n25), .Q(
        mdb_out[0]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_1_ ( .D(n196), .CP(mclk), .RN(n43), .Q(
        mdb_out[1]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_2_ ( .D(n195), .CP(mclk), .RN(n43), .Q(
        mdb_out[2]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_3_ ( .D(n194), .CP(mclk), .RN(n43), .Q(
        mdb_out[3]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_4_ ( .D(n193), .CP(mclk), .RN(n43), .Q(
        mdb_out[4]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_5_ ( .D(n192), .CP(mclk), .RN(n43), .Q(
        mdb_out[5]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_6_ ( .D(n191), .CP(mclk), .RN(n43), .Q(
        mdb_out[6]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_7_ ( .D(n190), .CP(mclk), .RN(n43), .Q(
        mdb_out[7]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_8_ ( .D(n189), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[8]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_9_ ( .D(n188), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[9]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_10_ ( .D(n187), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[10]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_11_ ( .D(n186), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[11]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_12_ ( .D(n185), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[12]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_13_ ( .D(n184), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[13]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_14_ ( .D(n183), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[14]) );
  HS65_GS_DFPRQX4 mdb_out_nxt_reg_15_ ( .D(n182), .CP(mclk), .RN(n43), .Q(
        mdb_out_nxt[15]) );
  HS65_GS_DFPRQX4 mdb_in_buf_en_reg ( .D(n178), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf_en) );
  HS65_GS_DFPRQX4 mdb_in_buf_valid_reg ( .D(n199), .CP(mclk), .RN(n25), .Q(
        mdb_in_buf_valid) );
  HS65_GS_IVX9 U3 ( .A(1'b1), .Z(scg1) );
  HS65_GS_IVX9 U5 ( .A(1'b1), .Z(scg0) );
  HS65_GS_IVX9 U7 ( .A(1'b1), .Z(oscoff) );
  HS65_GS_IVX9 U9 ( .A(1'b1), .Z(cpuoff) );
  HS65_GS_NOR4ABX4 U11 ( .A(n73), .B(n141), .C(n8), .D(n79), .Z(n34) );
  HS65_GS_NOR4ABX4 U12 ( .A(mdb_in_buf_valid), .B(n154), .C(n20), .D(n22), .Z(
        n37) );
  HS65_GS_IVX2 U13 ( .A(puc_rst), .Z(n25) );
  HS65_GS_BFX9 U14 ( .A(n213), .Z(pc_sw_wr) );
  HS65_GS_OAI31X5 U15 ( .A(n130), .B(n156), .C(n179), .D(n133), .Z(n94) );
  HS65_GS_NAND3X5 U16 ( .A(n84), .B(n85), .C(n86), .Z(n20) );
  HS65_GS_NOR4ABX2 U17 ( .A(n141), .B(n154), .C(n22), .D(mdb_in_buf_valid), 
        .Z(n30) );
  HS65_GS_AO12X9 U18 ( .A(n155), .B(n12), .C(dbg_reg_wr), .Z(reg_dest_wr) );
  HS65_GS_IVX9 U19 ( .A(n84), .Z(n181) );
  HS65_GS_IVX9 U20 ( .A(n6), .Z(n200) );
  HS65_GS_IVX9 U21 ( .A(n74), .Z(n160) );
  HS65_GS_BFX9 U22 ( .A(n25), .Z(n43) );
  HS65_GS_AOI12X2 U23 ( .A(mab[0]), .B(n40), .C(n157), .Z(mb_wr[0]) );
  HS65_GS_AOI12X2 U24 ( .A(n40), .B(n46), .C(n157), .Z(mb_wr[1]) );
  HS65_GS_IVX9 U25 ( .A(mab[0]), .Z(n46) );
  HS65_GS_IVX9 U26 ( .A(mdb_in[14]), .Z(n105) );
  HS65_GS_IVX9 U27 ( .A(mdb_in[13]), .Z(n111) );
  HS65_GS_IVX9 U28 ( .A(mdb_in[12]), .Z(n117) );
  HS65_GS_IVX9 U29 ( .A(mdb_in[11]), .Z(n121) );
  HS65_GS_IVX9 U30 ( .A(mdb_in[10]), .Z(n125) );
  HS65_GS_IVX9 U31 ( .A(mdb_in[15]), .Z(n99) );
  HS65_GS_IVX9 U32 ( .A(mdb_in[9]), .Z(n129) );
  HS65_GS_IVX9 U33 ( .A(mdb_in[8]), .Z(n132) );
  HS65_GS_IVX9 U34 ( .A(n116), .Z(n131) );
  HS65_GS_IVX9 U35 ( .A(n101), .Z(n108) );
  HS65_GS_IVX9 U36 ( .A(n104), .Z(n114) );
  HS65_GS_IVX9 U37 ( .A(n107), .Z(n119) );
  HS65_GS_IVX9 U38 ( .A(n110), .Z(n123) );
  HS65_GS_IVX9 U39 ( .A(n113), .Z(n127) );
  HS65_GS_IVX9 U40 ( .A(n98), .Z(n102) );
  HS65_GS_OAI211X5 U41 ( .A(n1), .B(n77), .C(n3), .D(n200), .Z(n22) );
  HS65_GS_NOR2AX3 U42 ( .A(n144), .B(dbg_halt_st), .Z(n133) );
  HS65_GS_IVX9 U43 ( .A(n30), .Z(n136) );
  HS65_GS_NAND2X7 U44 ( .A(n151), .B(n152), .Z(n85) );
  HS65_GS_IVX9 U45 ( .A(n8), .Z(n155) );
  HS65_GS_IVX9 U46 ( .A(n147), .Z(n137) );
  HS65_GS_NAND3X5 U47 ( .A(n153), .B(n201), .C(n151), .Z(n84) );
  HS65_GS_NAND2X7 U48 ( .A(n133), .B(n138), .Z(n93) );
  HS65_GS_NAND2X7 U49 ( .A(n152), .B(n159), .Z(n74) );
  HS65_GS_NOR2X6 U50 ( .A(n144), .B(dbg_halt_st), .Z(n89) );
  HS65_GS_IVX9 U51 ( .A(n75), .Z(n180) );
  HS65_GS_NOR2X6 U52 ( .A(n207), .B(n75), .Z(reg_sr_wr) );
  HS65_GS_NOR2X6 U53 ( .A(n75), .B(n1), .Z(n6) );
  HS65_GS_NAND2X7 U54 ( .A(n9), .B(n145), .Z(n87) );
  HS65_GS_IVX9 U55 ( .A(n20), .Z(n141) );
  HS65_GS_OA12X9 U56 ( .A(n5), .B(n9), .C(n77), .Z(n2) );
  HS65_GS_IVX9 U57 ( .A(n73), .Z(n154) );
  HS65_GS_IVX9 U58 ( .A(n40), .Z(n204) );
  HS65_GS_IVX9 U59 ( .A(n9), .Z(n178) );
  HS65_GS_NOR2X6 U60 ( .A(n149), .B(n180), .Z(n150) );
  HS65_GS_OAI211X5 U61 ( .A(n136), .B(n99), .C(n51), .D(n52), .Z(op_src[15])
         );
  HS65_GS_AOI22X6 U62 ( .A(inst_dext[15]), .B(n23), .C(mdb_in_buf[15]), .D(n37), .Z(n51) );
  HS65_GS_AOI222X2 U63 ( .A(reg_src[15]), .B(n20), .C(inst_sext[15]), .D(n34), 
        .E(dbg_reg_din[15]), .F(n22), .Z(n52) );
  HS65_GS_AO212X4 U64 ( .A(n89), .B(inst_sext[7]), .C(dbg_mem_dout[7]), .D(
        dbg_halt_st), .E(n97), .Z(op_dst[7]) );
  HS65_GS_OAI212X5 U65 ( .A(n98), .B(n93), .C(n81), .D(n92), .E(n94), .Z(n97)
         );
  HS65_GS_IVX9 U66 ( .A(dbg_reg_din[7]), .Z(n81) );
  HS65_GS_MX41X7 U67 ( .D0(dbg_mem_dout[0]), .S0(dbg_halt_st), .D1(n89), .S1(
        inst_sext[0]), .D2(n146), .S2(n70), .D3(n148), .S3(dbg_reg_din[0]), 
        .Z(op_dst[0]) );
  HS65_GS_IVX9 U68 ( .A(n93), .Z(n146) );
  HS65_GS_IVX9 U69 ( .A(n92), .Z(n148) );
  HS65_GS_OAI211X5 U70 ( .A(n136), .B(n125), .C(n66), .D(n67), .Z(op_src[10])
         );
  HS65_GS_AOI22X6 U71 ( .A(inst_dext[10]), .B(n23), .C(mdb_in_buf[10]), .D(n37), .Z(n66) );
  HS65_GS_AOI222X2 U72 ( .A(reg_src[10]), .B(n20), .C(inst_sext[10]), .D(n34), 
        .E(dbg_reg_din[10]), .F(n22), .Z(n67) );
  HS65_GS_OAI211X5 U73 ( .A(n136), .B(n105), .C(n54), .D(n55), .Z(op_src[14])
         );
  HS65_GS_AOI22X6 U74 ( .A(inst_dext[14]), .B(n23), .C(mdb_in_buf[14]), .D(n37), .Z(n54) );
  HS65_GS_AOI222X2 U75 ( .A(reg_src[14]), .B(n20), .C(inst_sext[14]), .D(n34), 
        .E(dbg_reg_din[14]), .F(n22), .Z(n55) );
  HS65_GS_OAI211X5 U76 ( .A(n136), .B(n129), .C(n18), .D(n19), .Z(op_src[9])
         );
  HS65_GS_AOI22X6 U77 ( .A(inst_dext[9]), .B(n23), .C(mdb_in_buf[9]), .D(n37), 
        .Z(n18) );
  HS65_GS_AOI222X2 U78 ( .A(reg_src[9]), .B(n20), .C(inst_sext[9]), .D(n34), 
        .E(dbg_reg_din[9]), .F(n22), .Z(n19) );
  HS65_GS_OAI211X5 U79 ( .A(n136), .B(n111), .C(n57), .D(n58), .Z(op_src[13])
         );
  HS65_GS_AOI22X6 U80 ( .A(inst_dext[13]), .B(n23), .C(mdb_in_buf[13]), .D(n37), .Z(n57) );
  HS65_GS_AOI222X2 U81 ( .A(reg_src[13]), .B(n20), .C(inst_sext[13]), .D(n34), 
        .E(dbg_reg_din[13]), .F(n22), .Z(n58) );
  HS65_GS_OAI211X5 U82 ( .A(n136), .B(n117), .C(n60), .D(n61), .Z(op_src[12])
         );
  HS65_GS_AOI22X6 U83 ( .A(inst_dext[12]), .B(n23), .C(mdb_in_buf[12]), .D(n37), .Z(n60) );
  HS65_GS_AOI222X2 U84 ( .A(reg_src[12]), .B(n20), .C(inst_sext[12]), .D(n34), 
        .E(dbg_reg_din[12]), .F(n22), .Z(n61) );
  HS65_GS_OAI211X5 U85 ( .A(n136), .B(n121), .C(n63), .D(n64), .Z(op_src[11])
         );
  HS65_GS_AOI22X6 U86 ( .A(inst_dext[11]), .B(n23), .C(mdb_in_buf[11]), .D(n37), .Z(n63) );
  HS65_GS_AOI222X2 U87 ( .A(reg_src[11]), .B(n20), .C(inst_sext[11]), .D(n34), 
        .E(dbg_reg_din[11]), .F(n22), .Z(n64) );
  HS65_GS_OAI211X5 U88 ( .A(n136), .B(n132), .C(n26), .D(n27), .Z(op_src[8])
         );
  HS65_GS_AOI22X6 U89 ( .A(inst_dext[8]), .B(n23), .C(mdb_in_buf[8]), .D(n37), 
        .Z(n26) );
  HS65_GS_AOI222X2 U90 ( .A(reg_src[8]), .B(n20), .C(inst_sext[8]), .D(n34), 
        .E(dbg_reg_din[8]), .F(n22), .Z(n27) );
  HS65_GS_AOI22X6 U91 ( .A(mdb_in[6]), .B(n147), .C(mdb_in[14]), .D(n137), .Z(
        n101) );
  HS65_GS_AOI22X6 U92 ( .A(mdb_in[5]), .B(n147), .C(mdb_in[13]), .D(n137), .Z(
        n104) );
  HS65_GS_AOI22X6 U93 ( .A(mdb_in[4]), .B(n147), .C(mdb_in[12]), .D(n137), .Z(
        n107) );
  HS65_GS_AOI22X6 U94 ( .A(mdb_in[2]), .B(n147), .C(mdb_in[10]), .D(n137), .Z(
        n113) );
  HS65_GS_AOI22X6 U95 ( .A(mdb_in[3]), .B(n147), .C(mdb_in[11]), .D(n137), .Z(
        n110) );
  HS65_GS_AOI22X6 U96 ( .A(mdb_in[7]), .B(n147), .C(mdb_in[15]), .D(n137), .Z(
        n98) );
  HS65_GS_AO212X4 U97 ( .A(n89), .B(inst_sext[4]), .C(dbg_mem_dout[4]), .D(
        dbg_halt_st), .E(n106), .Z(op_dst[4]) );
  HS65_GS_OAI212X5 U98 ( .A(n107), .B(n93), .C(n96), .D(n92), .E(n94), .Z(n106) );
  HS65_GS_IVX9 U99 ( .A(dbg_reg_din[4]), .Z(n96) );
  HS65_GS_AO212X4 U100 ( .A(n89), .B(inst_sext[6]), .C(dbg_mem_dout[6]), .D(
        dbg_halt_st), .E(n100), .Z(op_dst[6]) );
  HS65_GS_OAI212X5 U101 ( .A(n101), .B(n93), .C(n88), .D(n92), .E(n94), .Z(
        n100) );
  HS65_GS_IVX9 U102 ( .A(dbg_reg_din[6]), .Z(n88) );
  HS65_GS_AO212X4 U103 ( .A(n89), .B(inst_sext[2]), .C(dbg_mem_dout[2]), .D(
        dbg_halt_st), .E(n112), .Z(op_dst[2]) );
  HS65_GS_OAI212X5 U104 ( .A(n113), .B(n93), .C(n53), .D(n92), .E(n94), .Z(
        n112) );
  HS65_GS_IVX9 U105 ( .A(dbg_reg_din[2]), .Z(n53) );
  HS65_GS_AOI22X6 U106 ( .A(mdb_in[1]), .B(n147), .C(n137), .D(mdb_in[9]), .Z(
        n116) );
  HS65_GS_AO212X4 U107 ( .A(n89), .B(inst_sext[3]), .C(dbg_mem_dout[3]), .D(
        dbg_halt_st), .E(n109), .Z(op_dst[3]) );
  HS65_GS_OAI212X5 U108 ( .A(n110), .B(n93), .C(n50), .D(n92), .E(n94), .Z(
        n109) );
  HS65_GS_IVX9 U109 ( .A(dbg_reg_din[3]), .Z(n50) );
  HS65_GS_AO212X4 U110 ( .A(n89), .B(inst_sext[1]), .C(dbg_mem_dout[1]), .D(
        dbg_halt_st), .E(n115), .Z(op_dst[1]) );
  HS65_GS_OAI212X5 U111 ( .A(n116), .B(n93), .C(n56), .D(n92), .E(n94), .Z(
        n115) );
  HS65_GS_IVX9 U112 ( .A(dbg_reg_din[1]), .Z(n56) );
  HS65_GS_AO212X4 U113 ( .A(n89), .B(inst_sext[5]), .C(dbg_mem_dout[5]), .D(
        dbg_halt_st), .E(n103), .Z(op_dst[5]) );
  HS65_GS_OAI212X5 U114 ( .A(n104), .B(n93), .C(n91), .D(n92), .E(n94), .Z(
        n103) );
  HS65_GS_IVX9 U115 ( .A(dbg_reg_din[5]), .Z(n91) );
  HS65_GS_NAND2X7 U116 ( .A(n28), .B(n29), .Z(op_src[7]) );
  HS65_GS_AOI222X2 U117 ( .A(reg_src[7]), .B(n20), .C(inst_sext[7]), .D(n34), 
        .E(dbg_reg_din[7]), .F(n22), .Z(n28) );
  HS65_GS_AOI222X2 U118 ( .A(n30), .B(n102), .C(inst_dext[7]), .D(n23), .E(
        mdb_in_buf[7]), .F(n37), .Z(n29) );
  HS65_GS_AO212X4 U119 ( .A(n89), .B(inst_sext[15]), .C(dbg_mem_dout[15]), .D(
        dbg_halt_st), .E(n118), .Z(op_dst[15]) );
  HS65_GS_OAI212X5 U120 ( .A(n59), .B(n92), .C(n99), .D(n93), .E(n94), .Z(n118) );
  HS65_GS_IVX9 U121 ( .A(dbg_reg_din[15]), .Z(n59) );
  HS65_GS_NAND2X7 U122 ( .A(n68), .B(n69), .Z(op_src[0]) );
  HS65_GS_AOI222X2 U123 ( .A(reg_src[0]), .B(n20), .C(inst_sext[0]), .D(n34), 
        .E(dbg_reg_din[0]), .F(n22), .Z(n68) );
  HS65_GS_AOI222X2 U124 ( .A(n30), .B(n70), .C(inst_dext[0]), .D(n23), .E(
        mdb_in_buf[0]), .F(n37), .Z(n69) );
  HS65_GS_AO212X4 U125 ( .A(n89), .B(inst_sext[14]), .C(dbg_mem_dout[14]), .D(
        dbg_halt_st), .E(n120), .Z(op_dst[14]) );
  HS65_GS_OAI212X5 U126 ( .A(n62), .B(n92), .C(n105), .D(n93), .E(n94), .Z(
        n120) );
  HS65_GS_IVX9 U127 ( .A(dbg_reg_din[14]), .Z(n62) );
  HS65_GS_AO212X4 U128 ( .A(n89), .B(inst_sext[9]), .C(dbg_mem_dout[9]), .D(
        dbg_halt_st), .E(n90), .Z(op_dst[9]) );
  HS65_GS_OAI212X5 U129 ( .A(n80), .B(n92), .C(n129), .D(n93), .E(n94), .Z(n90) );
  HS65_GS_IVX9 U130 ( .A(dbg_reg_din[9]), .Z(n80) );
  HS65_GS_AO212X4 U131 ( .A(n89), .B(inst_sext[13]), .C(dbg_mem_dout[13]), .D(
        dbg_halt_st), .E(n122), .Z(op_dst[13]) );
  HS65_GS_OAI212X5 U132 ( .A(n65), .B(n92), .C(n111), .D(n93), .E(n94), .Z(
        n122) );
  HS65_GS_IVX9 U133 ( .A(dbg_reg_din[13]), .Z(n65) );
  HS65_GS_AO212X4 U134 ( .A(n89), .B(inst_sext[12]), .C(dbg_mem_dout[12]), .D(
        dbg_halt_st), .E(n124), .Z(op_dst[12]) );
  HS65_GS_OAI212X5 U135 ( .A(n71), .B(n92), .C(n117), .D(n93), .E(n94), .Z(
        n124) );
  HS65_GS_IVX9 U136 ( .A(dbg_reg_din[12]), .Z(n71) );
  HS65_GS_AO212X4 U137 ( .A(n89), .B(inst_sext[11]), .C(dbg_mem_dout[11]), .D(
        dbg_halt_st), .E(n126), .Z(op_dst[11]) );
  HS65_GS_OAI212X5 U138 ( .A(n76), .B(n92), .C(n121), .D(n93), .E(n94), .Z(
        n126) );
  HS65_GS_IVX9 U139 ( .A(dbg_reg_din[11]), .Z(n76) );
  HS65_GS_AO212X4 U140 ( .A(n89), .B(inst_sext[10]), .C(dbg_mem_dout[10]), .D(
        dbg_halt_st), .E(n128), .Z(op_dst[10]) );
  HS65_GS_OAI212X5 U141 ( .A(n78), .B(n92), .C(n125), .D(n93), .E(n94), .Z(
        n128) );
  HS65_GS_IVX9 U142 ( .A(dbg_reg_din[10]), .Z(n78) );
  HS65_GS_AO212X4 U143 ( .A(n89), .B(inst_sext[8]), .C(dbg_mem_dout[8]), .D(
        dbg_halt_st), .E(n95), .Z(op_dst[8]) );
  HS65_GS_OAI212X5 U144 ( .A(n49), .B(n92), .C(n132), .D(n93), .E(n94), .Z(n95) );
  HS65_GS_IVX9 U145 ( .A(dbg_reg_din[8]), .Z(n49) );
  HS65_GS_NAND2X7 U146 ( .A(n32), .B(n33), .Z(op_src[6]) );
  HS65_GS_AOI222X2 U147 ( .A(reg_src[6]), .B(n20), .C(inst_sext[6]), .D(n34), 
        .E(dbg_reg_din[6]), .F(n22), .Z(n32) );
  HS65_GS_AOI222X2 U148 ( .A(n30), .B(n108), .C(inst_dext[6]), .D(n23), .E(
        mdb_in_buf[6]), .F(n37), .Z(n33) );
  HS65_GS_NAND2X7 U149 ( .A(n47), .B(n48), .Z(op_src[1]) );
  HS65_GS_AOI222X2 U150 ( .A(reg_src[1]), .B(n20), .C(inst_sext[1]), .D(n34), 
        .E(dbg_reg_din[1]), .F(n22), .Z(n47) );
  HS65_GS_AOI222X2 U151 ( .A(n30), .B(n131), .C(inst_dext[1]), .D(n23), .E(
        mdb_in_buf[1]), .F(n37), .Z(n48) );
  HS65_GS_NAND2X7 U152 ( .A(n35), .B(n36), .Z(op_src[5]) );
  HS65_GS_AOI222X2 U153 ( .A(reg_src[5]), .B(n20), .C(inst_sext[5]), .D(n34), 
        .E(dbg_reg_din[5]), .F(n22), .Z(n35) );
  HS65_GS_AOI222X2 U154 ( .A(n30), .B(n114), .C(inst_dext[5]), .D(n23), .E(
        mdb_in_buf[5]), .F(n37), .Z(n36) );
  HS65_GS_NAND2X7 U155 ( .A(n38), .B(n39), .Z(op_src[4]) );
  HS65_GS_AOI222X2 U156 ( .A(reg_src[4]), .B(n20), .C(inst_sext[4]), .D(n34), 
        .E(dbg_reg_din[4]), .F(n22), .Z(n38) );
  HS65_GS_AOI222X2 U157 ( .A(n30), .B(n119), .C(inst_dext[4]), .D(n23), .E(
        mdb_in_buf[4]), .F(n37), .Z(n39) );
  HS65_GS_NAND2X7 U158 ( .A(n41), .B(n42), .Z(op_src[3]) );
  HS65_GS_AOI222X2 U159 ( .A(reg_src[3]), .B(n20), .C(inst_sext[3]), .D(n34), 
        .E(dbg_reg_din[3]), .F(n22), .Z(n41) );
  HS65_GS_AOI222X2 U160 ( .A(n30), .B(n123), .C(inst_dext[3]), .D(n23), .E(
        mdb_in_buf[3]), .F(n37), .Z(n42) );
  HS65_GS_NAND2X7 U161 ( .A(n44), .B(n45), .Z(op_src[2]) );
  HS65_GS_AOI222X2 U162 ( .A(reg_src[2]), .B(n20), .C(inst_sext[2]), .D(n34), 
        .E(dbg_reg_din[2]), .F(n22), .Z(n44) );
  HS65_GS_AOI222X2 U163 ( .A(n30), .B(n127), .C(inst_dext[2]), .D(n23), .E(
        mdb_in_buf[2]), .F(n37), .Z(n45) );
  HS65_GS_AO22X9 U164 ( .A(mdb_in[0]), .B(n147), .C(n137), .D(mdb_in[8]), .Z(
        n70) );
  HS65_GS_AO222X4 U165 ( .A(alu_out[15]), .B(n149), .C(mdb_out_nxt[15]), .D(
        n150), .E(pc_nxt[15]), .F(n180), .Z(n182) );
  HS65_GS_AO222X4 U166 ( .A(alu_out[8]), .B(n149), .C(mdb_out_nxt[8]), .D(n150), .E(pc_nxt[8]), .F(n180), .Z(n189) );
  HS65_GS_AO222X4 U167 ( .A(alu_out[14]), .B(n149), .C(mdb_out_nxt[14]), .D(
        n150), .E(pc_nxt[14]), .F(n180), .Z(n183) );
  HS65_GS_AO222X4 U168 ( .A(alu_out[13]), .B(n149), .C(mdb_out_nxt[13]), .D(
        n150), .E(pc_nxt[13]), .F(n180), .Z(n184) );
  HS65_GS_AO222X4 U169 ( .A(alu_out[12]), .B(n149), .C(mdb_out_nxt[12]), .D(
        n150), .E(pc_nxt[12]), .F(n180), .Z(n185) );
  HS65_GS_AO222X4 U170 ( .A(alu_out[11]), .B(n149), .C(mdb_out_nxt[11]), .D(
        n150), .E(pc_nxt[11]), .F(n180), .Z(n186) );
  HS65_GS_AO222X4 U171 ( .A(alu_out[10]), .B(n149), .C(mdb_out_nxt[10]), .D(
        n150), .E(pc_nxt[10]), .F(n180), .Z(n187) );
  HS65_GS_AO222X4 U172 ( .A(alu_out[9]), .B(n149), .C(mdb_out_nxt[9]), .D(n150), .E(pc_nxt[9]), .F(n180), .Z(n188) );
  HS65_GS_AO222X4 U173 ( .A(alu_out[7]), .B(n149), .C(mdb_out[7]), .D(n150), 
        .E(pc_nxt[7]), .F(n180), .Z(n190) );
  HS65_GS_IVX9 U174 ( .A(n85), .Z(n179) );
  HS65_GS_IVX9 U175 ( .A(n3), .Z(n156) );
  HS65_GS_OAI22X6 U176 ( .A(n1), .B(n2), .C(inst_so[6]), .D(n200), .Z(n130) );
  HS65_GS_NOR2X6 U177 ( .A(inst_so[5]), .B(inst_so[4]), .Z(n1) );
  HS65_GS_OAI32X5 U178 ( .A(n143), .B(inst_ad[0]), .C(n8), .D(n207), .E(n74), 
        .Z(n138) );
  HS65_GS_NAND3X5 U179 ( .A(n208), .B(n206), .C(n207), .Z(n143) );
  HS65_GS_NAND3X5 U180 ( .A(e_state[0]), .B(n159), .C(e_state[1]), .Z(n8) );
  HS65_GS_AOI32X5 U181 ( .A(n155), .B(n206), .C(inst_as[0]), .D(n87), .E(n202), 
        .Z(n86) );
  HS65_GS_IVX9 U182 ( .A(inst_as[6]), .Z(n202) );
  HS65_GS_NAND3AX6 U183 ( .A(n138), .B(n133), .C(n139), .Z(n92) );
  HS65_GS_OAI22X6 U184 ( .A(inst_ad[6]), .B(n74), .C(inst_so[6]), .D(n140), 
        .Z(n139) );
  HS65_GS_AOI13X5 U185 ( .A(n180), .B(n203), .C(n1), .D(n155), .Z(n140) );
  HS65_GS_IVX9 U186 ( .A(inst_ad[6]), .Z(n203) );
  HS65_GS_NAND3X5 U187 ( .A(inst_as[1]), .B(e_state[2]), .C(n134), .Z(n77) );
  HS65_GS_NOR3X4 U188 ( .A(n153), .B(e_state[3]), .C(e_state[1]), .Z(n134) );
  HS65_GS_NAND4ABX3 U189 ( .A(n201), .B(n153), .C(n209), .D(e_state[2]), .Z(
        n145) );
  HS65_GS_NAND3X5 U190 ( .A(e_state[2]), .B(n209), .C(n152), .Z(n9) );
  HS65_GS_NOR2X6 U191 ( .A(n209), .B(e_state[2]), .Z(n159) );
  HS65_GS_AND3X9 U192 ( .A(n1), .B(n72), .C(n73), .Z(n23) );
  HS65_GS_OAI21X3 U193 ( .A(inst_so[6]), .B(n74), .C(n75), .Z(n72) );
  HS65_GS_NOR2X6 U194 ( .A(e_state[3]), .B(e_state[2]), .Z(n151) );
  HS65_GS_NAND3X5 U195 ( .A(n159), .B(n201), .C(e_state[0]), .Z(n75) );
  HS65_GS_OAI31X5 U196 ( .A(inst_as[1]), .B(inst_as[6]), .C(inst_as[4]), .D(
        n87), .Z(n144) );
  HS65_GS_NAND2X7 U197 ( .A(n151), .B(e_state[0]), .Z(n3) );
  HS65_GS_IVX9 U198 ( .A(e_state[1]), .Z(n201) );
  HS65_GS_IVX9 U199 ( .A(inst_so[6]), .Z(n207) );
  HS65_GS_IVX9 U200 ( .A(e_state[3]), .Z(n209) );
  HS65_GS_NOR2X6 U201 ( .A(n201), .B(e_state[0]), .Z(n152) );
  HS65_GS_AOI12X2 U202 ( .A(n82), .B(n155), .C(reg_sr_wr), .Z(n73) );
  HS65_GS_OR3X9 U203 ( .A(n83), .B(inst_as[2]), .C(inst_as[1]), .Z(n82) );
  HS65_GS_OR3X9 U204 ( .A(inst_as[4]), .B(inst_as[6]), .C(inst_as[3]), .Z(n83)
         );
  HS65_GS_NAND2X7 U205 ( .A(mab_lsb), .B(n40), .Z(n147) );
  HS65_GS_IVX9 U206 ( .A(inst_type[1]), .Z(n206) );
  HS65_GS_OAI21X3 U207 ( .A(inst_as[3]), .B(inst_as[2]), .C(inst_src[1]), .Z(
        n5) );
  HS65_GS_IVX9 U208 ( .A(e_state[0]), .Z(n153) );
  HS65_GS_IVX9 U209 ( .A(inst_type[0]), .Z(n208) );
  HS65_GS_BFX9 U210 ( .A(inst_bw), .Z(n40) );
  HS65_GS_NOR4ABX2 U211 ( .A(n205), .B(n206), .C(inst_so[6]), .D(inst_as[7]), 
        .Z(n79) );
  HS65_GS_IVX9 U212 ( .A(inst_as[5]), .Z(n205) );
  HS65_GS_AO222X4 U213 ( .A(alu_out[6]), .B(n149), .C(mdb_out[6]), .D(n150), 
        .E(pc_nxt[6]), .F(n180), .Z(n191) );
  HS65_GS_AO222X4 U214 ( .A(alu_out[4]), .B(n149), .C(mdb_out[4]), .D(n150), 
        .E(pc_nxt[4]), .F(n180), .Z(n193) );
  HS65_GS_AO222X4 U215 ( .A(alu_out[3]), .B(n149), .C(mdb_out[3]), .D(n150), 
        .E(pc_nxt[3]), .F(n180), .Z(n194) );
  HS65_GS_AO222X4 U216 ( .A(alu_out[2]), .B(n149), .C(mdb_out[2]), .D(n150), 
        .E(pc_nxt[2]), .F(n180), .Z(n195) );
  HS65_GS_AO222X4 U217 ( .A(alu_out[1]), .B(n149), .C(mdb_out[1]), .D(n150), 
        .E(pc_nxt[1]), .F(n180), .Z(n196) );
  HS65_GS_AO222X4 U218 ( .A(alu_out[5]), .B(n149), .C(mdb_out[5]), .D(n150), 
        .E(pc_nxt[5]), .F(n180), .Z(n192) );
  HS65_GS_OAI311X5 U219 ( .A(n75), .B(inst_type[0]), .C(inst_mov), .D(n157), 
        .E(n158), .Z(mb_en) );
  HS65_GS_AOI22X6 U220 ( .A(n178), .B(n205), .C(n155), .D(inst_so[6]), .Z(n158) );
  HS65_GS_CBI4I1X5 U221 ( .A(n160), .B(n207), .C(n161), .D(n210), .Z(n157) );
  HS65_GS_OAI21X3 U222 ( .A(inst_irq_rst), .B(n3), .C(n145), .Z(n161) );
  HS65_GS_AO222X4 U223 ( .A(alu_out[0]), .B(n149), .C(mdb_out[0]), .D(n150), 
        .E(pc_nxt[0]), .F(n180), .Z(n197) );
  HS65_GS_IVX9 U224 ( .A(inst_alu[11]), .Z(n210) );
  HS65_GS_AO22X9 U225 ( .A(n40), .B(mdb_out[0]), .C(n204), .D(mdb_out_nxt[8]), 
        .Z(mdb_out[8]) );
  HS65_GS_AO22X9 U226 ( .A(n40), .B(mdb_out[1]), .C(n204), .D(mdb_out_nxt[9]), 
        .Z(mdb_out[9]) );
  HS65_GS_AO22X9 U227 ( .A(n40), .B(mdb_out[2]), .C(n204), .D(mdb_out_nxt[10]), 
        .Z(mdb_out[10]) );
  HS65_GS_AO22X9 U228 ( .A(n40), .B(mdb_out[3]), .C(n204), .D(mdb_out_nxt[11]), 
        .Z(mdb_out[11]) );
  HS65_GS_AO22X9 U229 ( .A(n40), .B(mdb_out[4]), .C(n204), .D(mdb_out_nxt[12]), 
        .Z(mdb_out[12]) );
  HS65_GS_AO22X9 U230 ( .A(n40), .B(mdb_out[5]), .C(n204), .D(mdb_out_nxt[13]), 
        .Z(mdb_out[13]) );
  HS65_GS_AO22X9 U231 ( .A(n40), .B(mdb_out[6]), .C(n204), .D(mdb_out_nxt[14]), 
        .Z(mdb_out[14]) );
  HS65_GS_AO22X9 U232 ( .A(n40), .B(mdb_out[7]), .C(n204), .D(mdb_out_nxt[15]), 
        .Z(mdb_out[15]) );
  HS65_GS_AO22X9 U233 ( .A(n135), .B(mdb_in_buf[10]), .C(mdb_in[10]), .D(
        mdb_in_buf_en), .Z(n172) );
  HS65_GS_AO22X9 U234 ( .A(n135), .B(mdb_in_buf[11]), .C(mdb_in[11]), .D(
        mdb_in_buf_en), .Z(n173) );
  HS65_GS_AO22X9 U235 ( .A(n135), .B(mdb_in_buf[12]), .C(mdb_in[12]), .D(
        mdb_in_buf_en), .Z(n174) );
  HS65_GS_AO22X9 U236 ( .A(n135), .B(mdb_in_buf[13]), .C(mdb_in[13]), .D(
        mdb_in_buf_en), .Z(n175) );
  HS65_GS_AO22X9 U237 ( .A(n135), .B(mdb_in_buf[14]), .C(mdb_in[14]), .D(
        mdb_in_buf_en), .Z(n176) );
  HS65_GS_AO22X9 U238 ( .A(n135), .B(mdb_in_buf[15]), .C(mdb_in[15]), .D(
        mdb_in_buf_en), .Z(n177) );
  HS65_GS_AO22X9 U239 ( .A(n135), .B(mdb_in_buf[9]), .C(mdb_in[9]), .D(
        mdb_in_buf_en), .Z(n171) );
  HS65_GS_AO22X9 U240 ( .A(n135), .B(mdb_in_buf[8]), .C(mdb_in[8]), .D(
        mdb_in_buf_en), .Z(n170) );
  HS65_GS_AO22X9 U241 ( .A(mb_en), .B(mab[0]), .C(n142), .D(mab_lsb), .Z(n198)
         );
  HS65_GS_IVX9 U242 ( .A(mb_en), .Z(n142) );
  HS65_GS_AO22X9 U243 ( .A(n135), .B(mdb_in_buf[2]), .C(n127), .D(
        mdb_in_buf_en), .Z(n163) );
  HS65_GS_AO22X9 U244 ( .A(n135), .B(mdb_in_buf[3]), .C(n123), .D(
        mdb_in_buf_en), .Z(n164) );
  HS65_GS_AO22X9 U245 ( .A(n135), .B(mdb_in_buf[4]), .C(n119), .D(
        mdb_in_buf_en), .Z(n165) );
  HS65_GS_AO22X9 U246 ( .A(n135), .B(mdb_in_buf[5]), .C(n114), .D(
        mdb_in_buf_en), .Z(n166) );
  HS65_GS_AO22X9 U247 ( .A(n135), .B(mdb_in_buf[6]), .C(n108), .D(
        mdb_in_buf_en), .Z(n167) );
  HS65_GS_AO22X9 U248 ( .A(n135), .B(mdb_in_buf[7]), .C(n102), .D(
        mdb_in_buf_en), .Z(n168) );
  HS65_GS_AO22X9 U249 ( .A(n135), .B(mdb_in_buf[1]), .C(n131), .D(
        mdb_in_buf_en), .Z(n162) );
  HS65_GS_AO22X9 U250 ( .A(n135), .B(mdb_in_buf[0]), .C(n70), .D(mdb_in_buf_en), .Z(n169) );
  HS65_GS_AO311X9 U251 ( .A(inst_ad[0]), .B(n210), .C(inst_type[2]), .D(
        inst_type[1]), .E(n14), .Z(n12) );
  HS65_GS_NOR4ABX2 U252 ( .A(n1), .B(inst_as[0]), .C(n208), .D(inst_so[6]), 
        .Z(n14) );
  HS65_GS_AO22X9 U253 ( .A(inst_so[6]), .B(n160), .C(n155), .D(inst_so[5]), 
        .Z(reg_pc_call) );
  HS65_GS_OAI212X5 U254 ( .A(n1), .B(n2), .C(inst_irq_rst), .D(n3), .E(n4), 
        .Z(reg_sp_wr) );
  HS65_GS_NAND3AX6 U255 ( .A(inst_as[1]), .B(n5), .C(n6), .Z(n4) );
  HS65_GS_OAI211X5 U256 ( .A(inst_so[5]), .B(n8), .C(n84), .D(n85), .Z(n149)
         );
  HS65_GS_IVX9 U257 ( .A(mdb_in_buf_en), .Z(n135) );
  HS65_GS_CBI4I1X5 U258 ( .A(n8), .B(n9), .C(n207), .D(n11), .Z(reg_incr) );
  HS65_GS_NAND2X7 U259 ( .A(inst_as[3]), .B(exec_done), .Z(n11) );
  HS65_GS_OA12X9 U260 ( .A(mdb_in_buf_valid), .B(mdb_in_buf_en), .C(n8), .Z(
        n199) );
endmodule


module omsp_mem_backbone ( cpu_halt_cmd, dbg_mem_din, dmem_addr, dmem_cen, 
        dmem_din, dmem_wen, eu_mdb_in, fe_mdb_in, fe_pmem_wait, dma_dout, 
        dma_ready, dma_resp, per_addr, per_din, per_we, per_en, pmem_addr, 
        pmem_cen, pmem_din, pmem_wen, cpu_halt_st, dbg_halt_cmd, dbg_mem_addr, 
        dbg_mem_dout, dbg_mem_en, dbg_mem_wr, dmem_dout, eu_mab, eu_mb_en, 
        eu_mb_wr, eu_mdb_out, fe_mab, fe_mb_en, mclk, dma_addr, dma_din, 
        dma_en, dma_priority, dma_we, per_dout, pmem_dout, puc_rst, 
        scan_enable );
  output [15:0] dbg_mem_din;
  output [12:0] dmem_addr;
  output [15:0] dmem_din;
  output [1:0] dmem_wen;
  output [15:0] eu_mdb_in;
  output [15:0] fe_mdb_in;
  output [15:0] dma_dout;
  output [13:0] per_addr;
  output [15:0] per_din;
  output [1:0] per_we;
  output [14:0] pmem_addr;
  output [15:0] pmem_din;
  output [1:0] pmem_wen;
  input [15:1] dbg_mem_addr;
  input [15:0] dbg_mem_dout;
  input [1:0] dbg_mem_wr;
  input [15:0] dmem_dout;
  input [14:0] eu_mab;
  input [1:0] eu_mb_wr;
  input [15:0] eu_mdb_out;
  input [14:0] fe_mab;
  input [15:1] dma_addr;
  input [15:0] dma_din;
  input [1:0] dma_we;
  input [15:0] per_dout;
  input [15:0] pmem_dout;
  input cpu_halt_st, dbg_halt_cmd, dbg_mem_en, eu_mb_en, fe_mb_en, mclk,
         dma_en, dma_priority, puc_rst, scan_enable;
  output cpu_halt_cmd, dmem_cen, fe_pmem_wait, dma_ready, dma_resp, per_en,
         pmem_cen;
  wire   N11, eu_dmem_addr_12_, eu_dmem_addr_11_, eu_dmem_addr_10_,
         eu_dmem_addr_9_, eu_dmem_addr_8_, ext_dmem_addr_12_,
         ext_dmem_addr_11_, ext_dmem_addr_10_, ext_dmem_addr_9_, eu_pmem_en,
         eu_pmem_addr_14_, fe_pmem_addr_14_, ext_pmem_en, ext_pmem_addr_14_,
         fe_pmem_en_dly, pmem_dout_bckup_sel, eu_mdb_in_sel_0_,
         ext_mem_din_sel_0_, n15, n17, n52, n84, n85, n87, n88, n90, n92, n93,
         n94, n112, n113, n115, n116, n117, n118, n119, n120, n121, n122, n123,
         n124, n125, n126, n127, n128, n129, n130, n131, n132, n133, n134,
         n136, n137, n138, n139, n140, n141, n142, n143, n144, n145, n146,
         n147, n148, n149, n150, n151, n152, n153, n157, n51, n53, n54, n55,
         n56, n57, n75, n76, n77, n78, n79, n80, n81, n82, n83, n86, n89, n91,
         n95, n96, n97, n98, n99, n100, n101, n102, n103, n104, n105, n106,
         n107, n108, n109, n110, n111, n114, n135, n154, n155, n156, n158,
         n159, n160, n161, n162, n163, n164, n165, n166, n167, n168, n169,
         n170, n171, n172, n173, n174, n175, n176, n177, n178, n179, n180,
         n181, n182, n183, n184, n185, n186, n187, n188, n189, n190, n191,
         n192, n193, n194, n195, n196, n197, n198, n199, n200, n201, n202,
         n203, n204, n205, n206, n207, n208, n209, n210, n211, n212, n213,
         n214, n215, n216, n217, n218, n219;
  wire   [15:0] pmem_dout_bckup;
  wire   [12:10] sub_239_carry;
  wire   [12:10] sub_230_carry;

  HS65_GS_DFPRQX9 ext_mem_din_sel_reg_0_ ( .D(n57), .CP(mclk), .RN(n97), .Q(
        ext_mem_din_sel_0_) );
  HS65_GS_DFPRQNX9 ext_mem_din_sel_reg_1_ ( .D(ext_pmem_en), .CP(mclk), .RN(
        n97), .QN(n153) );
  HS65_GS_DFPRQNX4 eu_mdb_in_sel_reg_1_ ( .D(eu_pmem_en), .CP(mclk), .RN(n56), 
        .QN(n152) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_15_ ( .D(per_dout[15]), .CP(mclk), .RN(n56), .QN(n136) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_13_ ( .D(per_dout[13]), .CP(mclk), .RN(n97), .QN(n138) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_12_ ( .D(per_dout[12]), .CP(mclk), .RN(n56), .QN(n139) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_11_ ( .D(per_dout[11]), .CP(mclk), .RN(n97), .QN(n140) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_10_ ( .D(per_dout[10]), .CP(mclk), .RN(n56), .QN(n141) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_8_ ( .D(per_dout[8]), .CP(mclk), .RN(n97), 
        .QN(n143) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_6_ ( .D(per_dout[6]), .CP(mclk), .RN(n56), 
        .QN(n145) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_4_ ( .D(per_dout[4]), .CP(mclk), .RN(n97), 
        .QN(n147) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_2_ ( .D(per_dout[2]), .CP(mclk), .RN(n56), 
        .QN(n149) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_14_ ( .D(per_dout[14]), .CP(mclk), .RN(n56), .QN(n137) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_9_ ( .D(per_dout[9]), .CP(mclk), .RN(n97), 
        .QN(n142) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_7_ ( .D(per_dout[7]), .CP(mclk), .RN(n97), 
        .QN(n144) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_5_ ( .D(per_dout[5]), .CP(mclk), .RN(n56), 
        .QN(n146) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_3_ ( .D(per_dout[3]), .CP(mclk), .RN(n56), 
        .QN(n148) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_1_ ( .D(per_dout[1]), .CP(mclk), .RN(n97), 
        .QN(n150) );
  HS65_GS_DFPRQNX4 per_dout_val_reg_0_ ( .D(per_dout[0]), .CP(mclk), .RN(n97), 
        .QN(n151) );
  HS65_GS_DFPRQX4 eu_mdb_in_sel_reg_0_ ( .D(n96), .CP(mclk), .RN(n97), .Q(
        eu_mdb_in_sel_0_) );
  HS65_GS_DFPRQX4 fe_pmem_en_dly_reg ( .D(n89), .CP(mclk), .RN(n97), .Q(
        fe_pmem_en_dly) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_sel_reg ( .D(n157), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup_sel) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_15_ ( .D(n134), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[15]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_14_ ( .D(n133), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[14]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_13_ ( .D(n132), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[13]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_12_ ( .D(n131), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[12]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_11_ ( .D(n130), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[11]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_10_ ( .D(n129), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[10]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_9_ ( .D(n128), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[9]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_8_ ( .D(n127), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[8]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_7_ ( .D(n126), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[7]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_6_ ( .D(n125), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[6]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_5_ ( .D(n124), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[5]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_4_ ( .D(n123), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[4]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_3_ ( .D(n122), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[3]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_2_ ( .D(n121), .CP(mclk), .RN(n56), .Q(
        pmem_dout_bckup[2]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_1_ ( .D(n120), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[1]) );
  HS65_GS_DFPRQX4 pmem_dout_bckup_reg_0_ ( .D(n119), .CP(mclk), .RN(n97), .Q(
        pmem_dout_bckup[0]) );
  HS65_GS_IVX9 U3 ( .A(1'b1), .Z(per_addr[8]) );
  HS65_GS_IVX9 U5 ( .A(1'b1), .Z(per_addr[9]) );
  HS65_GS_IVX9 U7 ( .A(1'b1), .Z(per_addr[10]) );
  HS65_GS_IVX9 U9 ( .A(1'b1), .Z(per_addr[11]) );
  HS65_GS_IVX9 U11 ( .A(1'b1), .Z(per_addr[12]) );
  HS65_GS_IVX9 U13 ( .A(1'b1), .Z(per_addr[13]) );
  HS65_GS_IVX9 U15 ( .A(1'b0), .Z(dma_resp) );
  HS65_GS_IVX9 U17 ( .A(1'b0), .Z(dma_ready) );
  HS65_GS_IVX9 U19 ( .A(1'b1), .Z(dma_dout[0]) );
  HS65_GS_IVX9 U21 ( .A(1'b1), .Z(dma_dout[1]) );
  HS65_GS_IVX9 U23 ( .A(1'b1), .Z(dma_dout[2]) );
  HS65_GS_IVX9 U25 ( .A(1'b1), .Z(dma_dout[3]) );
  HS65_GS_IVX9 U27 ( .A(1'b1), .Z(dma_dout[4]) );
  HS65_GS_IVX9 U29 ( .A(1'b1), .Z(dma_dout[5]) );
  HS65_GS_IVX9 U31 ( .A(1'b1), .Z(dma_dout[6]) );
  HS65_GS_IVX9 U33 ( .A(1'b1), .Z(dma_dout[7]) );
  HS65_GS_IVX9 U35 ( .A(1'b1), .Z(dma_dout[8]) );
  HS65_GS_IVX9 U37 ( .A(1'b1), .Z(dma_dout[9]) );
  HS65_GS_IVX9 U39 ( .A(1'b1), .Z(dma_dout[10]) );
  HS65_GS_IVX9 U41 ( .A(1'b1), .Z(dma_dout[11]) );
  HS65_GS_IVX9 U43 ( .A(1'b1), .Z(dma_dout[12]) );
  HS65_GS_IVX9 U45 ( .A(1'b1), .Z(dma_dout[13]) );
  HS65_GS_IVX9 U47 ( .A(1'b1), .Z(dma_dout[14]) );
  HS65_GS_IVX9 U49 ( .A(1'b1), .Z(dma_dout[15]) );
  HS65_GS_OAI222X2 U51 ( .A(eu_mab[13]), .B(n51), .C(fe_mab[13]), .D(n53), .E(
        dbg_mem_addr[14]), .F(n54), .Z(pmem_addr[13]) );
  HS65_GS_IVX9 U52 ( .A(eu_pmem_en), .Z(n51) );
  HS65_GS_IVX9 U53 ( .A(n15), .Z(n53) );
  HS65_GS_IVX9 U54 ( .A(ext_pmem_en), .Z(n54) );
  HS65_GS_MUX21I1X6 U55 ( .D0(dbg_mem_addr[9]), .D1(eu_dmem_addr_8_), .S0(n78), 
        .Z(dmem_addr[8]) );
  HS65_GS_AO112X9 U56 ( .A(dbg_mem_addr[13]), .B(n86), .C(dbg_mem_addr[15]), 
        .D(dbg_mem_addr[14]), .Z(n55) );
  HS65_GS_IVX2 U57 ( .A(puc_rst), .Z(n56) );
  HS65_GS_BFX9 U58 ( .A(n57), .Z(n81) );
  HS65_GS_NAND3X5 U59 ( .A(n88), .B(n82), .C(fe_pmem_en_dly), .Z(n52) );
  HS65_GS_NOR4ABX2 U60 ( .A(n92), .B(n116), .C(eu_mab[11]), .D(eu_mab[10]), 
        .Z(n85) );
  HS65_GS_NOR3X4 U61 ( .A(eu_mab[12]), .B(eu_mab[9]), .C(eu_mab[8]), .Z(n116)
         );
  HS65_GS_NOR2X6 U62 ( .A(eu_mab[13]), .B(eu_mab[14]), .Z(n92) );
  HS65_GS_NAND2X7 U63 ( .A(n17), .B(n80), .Z(per_en) );
  HS65_GS_NOR2AX3 U64 ( .A(n15), .B(n89), .Z(pmem_cen) );
  HS65_GS_AND2X4 U65 ( .A(eu_pmem_en), .B(n89), .Z(fe_pmem_wait) );
  HS65_GS_IVX9 U66 ( .A(n17), .Z(n96) );
  HS65_GS_NOR2AX3 U67 ( .A(n113), .B(n76), .Z(dmem_cen) );
  HS65_GS_IVX9 U68 ( .A(n81), .Z(n79) );
  HS65_GS_IVX9 U69 ( .A(n78), .Z(n75) );
  HS65_GS_IVX9 U70 ( .A(n78), .Z(n76) );
  HS65_GS_IVX9 U71 ( .A(n81), .Z(n80) );
  HS65_GS_BFX9 U72 ( .A(n78), .Z(n77) );
  HS65_GS_NAND2X7 U73 ( .A(n85), .B(eu_mb_en), .Z(n17) );
  HS65_GS_IVX9 U74 ( .A(n52), .Z(n91) );
  HS65_GS_OAI22X6 U75 ( .A(n91), .B(n135), .C(n52), .D(n219), .Z(n119) );
  HS65_GS_OAI22X6 U76 ( .A(n91), .B(n114), .C(n52), .D(n218), .Z(n120) );
  HS65_GS_OAI22X6 U77 ( .A(n91), .B(n111), .C(n52), .D(n217), .Z(n121) );
  HS65_GS_OAI22X6 U78 ( .A(n91), .B(n110), .C(n52), .D(n216), .Z(n122) );
  HS65_GS_OAI22X6 U79 ( .A(n91), .B(n109), .C(n52), .D(n215), .Z(n123) );
  HS65_GS_OAI22X6 U80 ( .A(n91), .B(n108), .C(n52), .D(n214), .Z(n124) );
  HS65_GS_OAI22X6 U81 ( .A(n91), .B(n107), .C(n52), .D(n213), .Z(n125) );
  HS65_GS_OAI22X6 U82 ( .A(n91), .B(n106), .C(n52), .D(n212), .Z(n126) );
  HS65_GS_OAI22X6 U83 ( .A(n91), .B(n105), .C(n52), .D(n211), .Z(n127) );
  HS65_GS_OAI22X6 U84 ( .A(n91), .B(n104), .C(n52), .D(n210), .Z(n128) );
  HS65_GS_OAI22X6 U85 ( .A(n91), .B(n103), .C(n52), .D(n209), .Z(n129) );
  HS65_GS_OAI22X6 U86 ( .A(n91), .B(n102), .C(n52), .D(n208), .Z(n130) );
  HS65_GS_OAI22X6 U87 ( .A(n91), .B(n101), .C(n52), .D(n207), .Z(n131) );
  HS65_GS_OAI22X6 U88 ( .A(n91), .B(n100), .C(n52), .D(n206), .Z(n132) );
  HS65_GS_OAI22X6 U89 ( .A(n91), .B(n99), .C(n52), .D(n205), .Z(n133) );
  HS65_GS_OAI22X6 U90 ( .A(n91), .B(n98), .C(n52), .D(n204), .Z(n134) );
  HS65_GS_IVX9 U91 ( .A(n88), .Z(n89) );
  HS65_GS_NOR4ABX4 U92 ( .A(eu_mb_en), .B(n95), .C(eu_mb_wr[0]), .D(
        eu_mb_wr[1]), .Z(eu_pmem_en) );
  HS65_GS_IVX9 U93 ( .A(n92), .Z(n95) );
  HS65_GS_NOR2X6 U94 ( .A(eu_pmem_en), .B(ext_pmem_en), .Z(n15) );
  HS65_GS_NAND3AX6 U95 ( .A(n85), .B(eu_mb_en), .C(N11), .Z(n113) );
  HS65_GS_OAI22X6 U96 ( .A(n81), .B(n155), .C(n79), .D(n172), .Z(per_din[0])
         );
  HS65_GS_OAI22X6 U97 ( .A(n81), .B(n158), .C(n79), .D(n173), .Z(per_din[1])
         );
  HS65_GS_OAI22X6 U98 ( .A(n81), .B(n160), .C(n79), .D(n174), .Z(per_din[2])
         );
  HS65_GS_OAI22X6 U99 ( .A(n81), .B(n162), .C(n79), .D(n175), .Z(per_din[3])
         );
  HS65_GS_OAI22X6 U100 ( .A(n81), .B(n164), .C(n79), .D(n176), .Z(per_din[4])
         );
  HS65_GS_OAI22X6 U101 ( .A(n81), .B(n166), .C(n79), .D(n177), .Z(per_din[5])
         );
  HS65_GS_OAI22X6 U102 ( .A(n81), .B(n168), .C(n79), .D(n178), .Z(per_din[6])
         );
  HS65_GS_OAI22X6 U103 ( .A(n81), .B(n170), .C(n80), .D(n179), .Z(per_din[7])
         );
  HS65_GS_OAI22X6 U104 ( .A(n81), .B(n156), .C(n80), .D(n180), .Z(per_din[8])
         );
  HS65_GS_OAI22X6 U105 ( .A(n81), .B(n159), .C(n80), .D(n181), .Z(per_din[9])
         );
  HS65_GS_OAI22X6 U106 ( .A(n81), .B(n161), .C(n79), .D(n182), .Z(per_din[10])
         );
  HS65_GS_OAI22X6 U107 ( .A(n81), .B(n163), .C(n79), .D(n183), .Z(per_din[11])
         );
  HS65_GS_OAI22X6 U108 ( .A(n81), .B(n165), .C(n79), .D(n184), .Z(per_din[12])
         );
  HS65_GS_OAI22X6 U109 ( .A(n81), .B(n167), .C(n79), .D(n185), .Z(per_din[13])
         );
  HS65_GS_OAI22X6 U110 ( .A(n81), .B(n169), .C(n79), .D(n186), .Z(per_din[14])
         );
  HS65_GS_OAI22X6 U111 ( .A(n81), .B(n171), .C(n79), .D(n187), .Z(per_din[15])
         );
  HS65_GS_IVX9 U112 ( .A(cpu_halt_st), .Z(n82) );
  HS65_GS_OAI22X6 U113 ( .A(n172), .B(n77), .C(n75), .D(n155), .Z(dmem_din[0])
         );
  HS65_GS_OAI22X6 U114 ( .A(n173), .B(n77), .C(n75), .D(n158), .Z(dmem_din[1])
         );
  HS65_GS_OAI22X6 U115 ( .A(n174), .B(n77), .C(n75), .D(n160), .Z(dmem_din[2])
         );
  HS65_GS_OAI22X6 U116 ( .A(n175), .B(n77), .C(n75), .D(n162), .Z(dmem_din[3])
         );
  HS65_GS_OAI22X6 U117 ( .A(n176), .B(n77), .C(n75), .D(n164), .Z(dmem_din[4])
         );
  HS65_GS_OAI22X6 U118 ( .A(n177), .B(n77), .C(n75), .D(n166), .Z(dmem_din[5])
         );
  HS65_GS_OAI22X6 U119 ( .A(n178), .B(n77), .C(n75), .D(n168), .Z(dmem_din[6])
         );
  HS65_GS_OAI22X6 U120 ( .A(n179), .B(n77), .C(n76), .D(n170), .Z(dmem_din[7])
         );
  HS65_GS_OAI22X6 U121 ( .A(n180), .B(n77), .C(n76), .D(n156), .Z(dmem_din[8])
         );
  HS65_GS_OAI22X6 U122 ( .A(n181), .B(n77), .C(n76), .D(n159), .Z(dmem_din[9])
         );
  HS65_GS_OAI22X6 U123 ( .A(n182), .B(n77), .C(n75), .D(n161), .Z(dmem_din[10]) );
  HS65_GS_OAI22X6 U124 ( .A(n183), .B(n77), .C(n75), .D(n163), .Z(dmem_din[11]) );
  HS65_GS_OAI22X6 U125 ( .A(n184), .B(n77), .C(n75), .D(n165), .Z(dmem_din[12]) );
  HS65_GS_OAI22X6 U126 ( .A(n185), .B(n77), .C(n75), .D(n167), .Z(dmem_din[13]) );
  HS65_GS_OAI22X6 U127 ( .A(n186), .B(n77), .C(n75), .D(n169), .Z(dmem_din[14]) );
  HS65_GS_OAI22X6 U128 ( .A(n187), .B(n77), .C(n75), .D(n171), .Z(dmem_din[15]) );
  HS65_GS_IVX9 U129 ( .A(puc_rst), .Z(n97) );
  HS65_GS_OAI222X2 U130 ( .A(n151), .B(n117), .C(n203), .D(n118), .E(n153), 
        .F(n219), .Z(dbg_mem_din[0]) );
  HS65_GS_OAI222X2 U131 ( .A(n141), .B(n117), .C(n193), .D(n118), .E(n153), 
        .F(n209), .Z(dbg_mem_din[10]) );
  HS65_GS_OAI222X2 U132 ( .A(n140), .B(n117), .C(n192), .D(n118), .E(n153), 
        .F(n208), .Z(dbg_mem_din[11]) );
  HS65_GS_OAI222X2 U133 ( .A(n139), .B(n117), .C(n191), .D(n118), .E(n153), 
        .F(n207), .Z(dbg_mem_din[12]) );
  HS65_GS_OAI222X2 U134 ( .A(n138), .B(n117), .C(n190), .D(n118), .E(n153), 
        .F(n206), .Z(dbg_mem_din[13]) );
  HS65_GS_OAI222X2 U135 ( .A(n137), .B(n117), .C(n189), .D(n118), .E(n153), 
        .F(n205), .Z(dbg_mem_din[14]) );
  HS65_GS_OAI222X2 U136 ( .A(n136), .B(n117), .C(n188), .D(n118), .E(n153), 
        .F(n204), .Z(dbg_mem_din[15]) );
  HS65_GS_OAI222X2 U137 ( .A(n150), .B(n117), .C(n202), .D(n118), .E(n153), 
        .F(n218), .Z(dbg_mem_din[1]) );
  HS65_GS_OAI222X2 U138 ( .A(n149), .B(n117), .C(n201), .D(n118), .E(n153), 
        .F(n217), .Z(dbg_mem_din[2]) );
  HS65_GS_OAI222X2 U139 ( .A(n148), .B(n117), .C(n200), .D(n118), .E(n153), 
        .F(n216), .Z(dbg_mem_din[3]) );
  HS65_GS_OAI222X2 U140 ( .A(n147), .B(n117), .C(n199), .D(n118), .E(n153), 
        .F(n215), .Z(dbg_mem_din[4]) );
  HS65_GS_OAI222X2 U141 ( .A(n146), .B(n117), .C(n198), .D(n118), .E(n153), 
        .F(n214), .Z(dbg_mem_din[5]) );
  HS65_GS_OAI222X2 U142 ( .A(n145), .B(n117), .C(n197), .D(n118), .E(n153), 
        .F(n213), .Z(dbg_mem_din[6]) );
  HS65_GS_OAI222X2 U143 ( .A(n144), .B(n117), .C(n196), .D(n118), .E(n153), 
        .F(n212), .Z(dbg_mem_din[7]) );
  HS65_GS_OAI222X2 U144 ( .A(n143), .B(n117), .C(n195), .D(n118), .E(n153), 
        .F(n211), .Z(dbg_mem_din[8]) );
  HS65_GS_OAI222X2 U145 ( .A(n142), .B(n117), .C(n194), .D(n118), .E(n153), 
        .F(n210), .Z(dbg_mem_din[9]) );
  HS65_GS_IVX9 U146 ( .A(n112), .Z(n78) );
  HS65_GS_AO22X9 U147 ( .A(eu_mab[0]), .B(n80), .C(dbg_mem_addr[1]), .D(n81), 
        .Z(per_addr[0]) );
  HS65_GS_OAI222X2 U148 ( .A(n137), .B(n93), .C(n94), .D(n189), .E(n152), .F(
        n205), .Z(eu_mdb_in[14]) );
  HS65_GS_OAI222X2 U149 ( .A(n138), .B(n93), .C(n94), .D(n190), .E(n152), .F(
        n206), .Z(eu_mdb_in[13]) );
  HS65_GS_OAI222X2 U150 ( .A(n139), .B(n93), .C(n94), .D(n191), .E(n152), .F(
        n207), .Z(eu_mdb_in[12]) );
  HS65_GS_OAI222X2 U151 ( .A(n141), .B(n93), .C(n94), .D(n193), .E(n152), .F(
        n209), .Z(eu_mdb_in[10]) );
  HS65_GS_OAI222X2 U152 ( .A(n140), .B(n93), .C(n94), .D(n192), .E(n152), .F(
        n208), .Z(eu_mdb_in[11]) );
  HS65_GS_OAI222X2 U153 ( .A(n136), .B(n93), .C(n94), .D(n188), .E(n152), .F(
        n204), .Z(eu_mdb_in[15]) );
  HS65_GS_OAI222X2 U154 ( .A(n142), .B(n93), .C(n94), .D(n194), .E(n152), .F(
        n210), .Z(eu_mdb_in[9]) );
  HS65_GS_OAI222X2 U155 ( .A(n143), .B(n93), .C(n94), .D(n195), .E(n152), .F(
        n211), .Z(eu_mdb_in[8]) );
  HS65_GS_AO22X9 U156 ( .A(eu_mab[2]), .B(n80), .C(dbg_mem_addr[3]), .D(n57), 
        .Z(per_addr[2]) );
  HS65_GS_OAI222X2 U157 ( .A(n145), .B(n93), .C(n94), .D(n197), .E(n152), .F(
        n213), .Z(eu_mdb_in[6]) );
  HS65_GS_OAI222X2 U158 ( .A(n146), .B(n93), .C(n94), .D(n198), .E(n152), .F(
        n214), .Z(eu_mdb_in[5]) );
  HS65_GS_OAI222X2 U159 ( .A(n147), .B(n93), .C(n94), .D(n199), .E(n152), .F(
        n215), .Z(eu_mdb_in[4]) );
  HS65_GS_OAI222X2 U160 ( .A(n149), .B(n93), .C(n94), .D(n201), .E(n152), .F(
        n217), .Z(eu_mdb_in[2]) );
  HS65_GS_OAI222X2 U161 ( .A(n148), .B(n93), .C(n94), .D(n200), .E(n152), .F(
        n216), .Z(eu_mdb_in[3]) );
  HS65_GS_OAI222X2 U162 ( .A(n144), .B(n93), .C(n94), .D(n196), .E(n152), .F(
        n212), .Z(eu_mdb_in[7]) );
  HS65_GS_OAI222X2 U163 ( .A(n150), .B(n93), .C(n94), .D(n202), .E(n152), .F(
        n218), .Z(eu_mdb_in[1]) );
  HS65_GS_AO22X9 U164 ( .A(eu_mab[1]), .B(n80), .C(dbg_mem_addr[2]), .D(n57), 
        .Z(per_addr[1]) );
  HS65_GS_AO222X4 U165 ( .A(eu_pmem_addr_14_), .B(eu_pmem_en), .C(
        fe_pmem_addr_14_), .D(n15), .E(ext_pmem_addr_14_), .F(ext_pmem_en), 
        .Z(pmem_addr[14]) );
  HS65_GS_AO22X9 U166 ( .A(n80), .B(eu_mb_wr[1]), .C(dbg_mem_wr[1]), .D(n57), 
        .Z(per_we[1]) );
  HS65_GS_IVX9 U167 ( .A(pmem_dout[0]), .Z(n219) );
  HS65_GS_IVX9 U168 ( .A(pmem_dout[10]), .Z(n209) );
  HS65_GS_IVX9 U169 ( .A(pmem_dout[11]), .Z(n208) );
  HS65_GS_IVX9 U170 ( .A(pmem_dout[12]), .Z(n207) );
  HS65_GS_IVX9 U171 ( .A(pmem_dout[13]), .Z(n206) );
  HS65_GS_IVX9 U172 ( .A(pmem_dout[14]), .Z(n205) );
  HS65_GS_IVX9 U173 ( .A(pmem_dout[15]), .Z(n204) );
  HS65_GS_IVX9 U174 ( .A(pmem_dout[1]), .Z(n218) );
  HS65_GS_IVX9 U175 ( .A(pmem_dout[2]), .Z(n217) );
  HS65_GS_IVX9 U176 ( .A(pmem_dout[3]), .Z(n216) );
  HS65_GS_IVX9 U177 ( .A(pmem_dout[4]), .Z(n215) );
  HS65_GS_IVX9 U178 ( .A(pmem_dout[5]), .Z(n214) );
  HS65_GS_IVX9 U179 ( .A(pmem_dout[6]), .Z(n213) );
  HS65_GS_IVX9 U180 ( .A(pmem_dout[7]), .Z(n212) );
  HS65_GS_IVX9 U181 ( .A(pmem_dout[8]), .Z(n211) );
  HS65_GS_IVX9 U182 ( .A(pmem_dout[9]), .Z(n210) );
  HS65_GS_AO22X9 U183 ( .A(eu_mab[5]), .B(n80), .C(dbg_mem_addr[6]), .D(n57), 
        .Z(per_addr[5]) );
  HS65_GS_OAI222X2 U184 ( .A(n151), .B(n93), .C(n94), .D(n203), .E(n152), .F(
        n219), .Z(eu_mdb_in[0]) );
  HS65_GS_AO22X9 U185 ( .A(eu_mab[6]), .B(n80), .C(dbg_mem_addr[7]), .D(n57), 
        .Z(per_addr[6]) );
  HS65_GS_AO22X9 U186 ( .A(n80), .B(eu_mb_wr[0]), .C(dbg_mem_wr[0]), .D(n81), 
        .Z(per_we[0]) );
  HS65_GS_AO22X9 U187 ( .A(eu_mab[7]), .B(n80), .C(dbg_mem_addr[8]), .D(n57), 
        .Z(per_addr[7]) );
  HS65_GS_AO22X9 U188 ( .A(eu_mab[4]), .B(n80), .C(dbg_mem_addr[5]), .D(n57), 
        .Z(per_addr[4]) );
  HS65_GS_IVX9 U189 ( .A(dmem_dout[0]), .Z(n203) );
  HS65_GS_IVX9 U190 ( .A(dmem_dout[10]), .Z(n193) );
  HS65_GS_IVX9 U191 ( .A(dmem_dout[11]), .Z(n192) );
  HS65_GS_IVX9 U192 ( .A(dmem_dout[12]), .Z(n191) );
  HS65_GS_IVX9 U193 ( .A(dmem_dout[13]), .Z(n190) );
  HS65_GS_IVX9 U194 ( .A(dmem_dout[14]), .Z(n189) );
  HS65_GS_IVX9 U195 ( .A(dmem_dout[15]), .Z(n188) );
  HS65_GS_IVX9 U196 ( .A(dmem_dout[1]), .Z(n202) );
  HS65_GS_IVX9 U197 ( .A(dmem_dout[2]), .Z(n201) );
  HS65_GS_IVX9 U198 ( .A(dmem_dout[3]), .Z(n200) );
  HS65_GS_IVX9 U199 ( .A(dmem_dout[4]), .Z(n199) );
  HS65_GS_IVX9 U200 ( .A(dmem_dout[5]), .Z(n198) );
  HS65_GS_IVX9 U201 ( .A(dmem_dout[6]), .Z(n197) );
  HS65_GS_IVX9 U202 ( .A(dmem_dout[7]), .Z(n196) );
  HS65_GS_IVX9 U203 ( .A(dmem_dout[8]), .Z(n195) );
  HS65_GS_IVX9 U204 ( .A(dmem_dout[9]), .Z(n194) );
  HS65_GS_AO22X9 U205 ( .A(eu_mab[3]), .B(n80), .C(dbg_mem_addr[4]), .D(n57), 
        .Z(per_addr[3]) );
  HS65_GS_OAI22X6 U206 ( .A(n100), .B(n154), .C(pmem_dout_bckup_sel), .D(n206), 
        .Z(fe_mdb_in[13]) );
  HS65_GS_OAI22X6 U207 ( .A(n104), .B(n154), .C(pmem_dout_bckup_sel), .D(n210), 
        .Z(fe_mdb_in[9]) );
  HS65_GS_OAI22X6 U208 ( .A(n99), .B(n154), .C(pmem_dout_bckup_sel), .D(n205), 
        .Z(fe_mdb_in[14]) );
  HS65_GS_OAI22X6 U209 ( .A(n98), .B(n154), .C(pmem_dout_bckup_sel), .D(n204), 
        .Z(fe_mdb_in[15]) );
  HS65_GS_OAI22X6 U210 ( .A(n111), .B(n154), .C(pmem_dout_bckup_sel), .D(n217), 
        .Z(fe_mdb_in[2]) );
  HS65_GS_OAI22X6 U211 ( .A(n135), .B(n154), .C(pmem_dout_bckup_sel), .D(n219), 
        .Z(fe_mdb_in[0]) );
  HS65_GS_OAI22X6 U212 ( .A(n105), .B(n154), .C(pmem_dout_bckup_sel), .D(n211), 
        .Z(fe_mdb_in[8]) );
  HS65_GS_OAI22X6 U213 ( .A(n110), .B(n154), .C(pmem_dout_bckup_sel), .D(n216), 
        .Z(fe_mdb_in[3]) );
  HS65_GS_OAI22X6 U214 ( .A(n114), .B(n154), .C(pmem_dout_bckup_sel), .D(n218), 
        .Z(fe_mdb_in[1]) );
  HS65_GS_OAI22X6 U215 ( .A(n103), .B(n154), .C(pmem_dout_bckup_sel), .D(n209), 
        .Z(fe_mdb_in[10]) );
  HS65_GS_OAI22X6 U216 ( .A(n102), .B(n154), .C(pmem_dout_bckup_sel), .D(n208), 
        .Z(fe_mdb_in[11]) );
  HS65_GS_OAI21X3 U217 ( .A(fe_mab[14]), .B(fe_mab[13]), .C(fe_mb_en), .Z(n88)
         );
  HS65_GS_OAI21X3 U218 ( .A(n154), .B(n87), .C(n52), .Z(n157) );
  HS65_GS_OAI21X3 U219 ( .A(fe_pmem_en_dly), .B(n88), .C(n82), .Z(n87) );
  HS65_GS_AO222X4 U220 ( .A(eu_mab[8]), .B(eu_pmem_en), .C(fe_mab[8]), .D(n15), 
        .E(dbg_mem_addr[9]), .F(ext_pmem_en), .Z(pmem_addr[8]) );
  HS65_GS_AO222X4 U221 ( .A(eu_mab[10]), .B(eu_pmem_en), .C(fe_mab[10]), .D(
        n15), .E(dbg_mem_addr[11]), .F(ext_pmem_en), .Z(pmem_addr[10]) );
  HS65_GS_AO222X4 U222 ( .A(eu_mab[11]), .B(eu_pmem_en), .C(fe_mab[11]), .D(
        n15), .E(dbg_mem_addr[12]), .F(ext_pmem_en), .Z(pmem_addr[11]) );
  HS65_GS_AO222X4 U223 ( .A(eu_mab[9]), .B(eu_pmem_en), .C(fe_mab[9]), .D(n15), 
        .E(dbg_mem_addr[10]), .F(ext_pmem_en), .Z(pmem_addr[9]) );
  HS65_GS_AO222X4 U224 ( .A(eu_mab[12]), .B(eu_pmem_en), .C(fe_mab[12]), .D(
        n15), .E(dbg_mem_addr[13]), .F(ext_pmem_en), .Z(pmem_addr[12]) );
  HS65_GS_AO222X4 U225 ( .A(eu_mab[1]), .B(eu_pmem_en), .C(fe_mab[1]), .D(n15), 
        .E(dbg_mem_addr[2]), .F(ext_pmem_en), .Z(pmem_addr[1]) );
  HS65_GS_AO222X4 U226 ( .A(eu_mab[2]), .B(eu_pmem_en), .C(fe_mab[2]), .D(n15), 
        .E(dbg_mem_addr[3]), .F(ext_pmem_en), .Z(pmem_addr[2]) );
  HS65_GS_AO222X4 U227 ( .A(eu_mab[3]), .B(eu_pmem_en), .C(fe_mab[3]), .D(n15), 
        .E(dbg_mem_addr[4]), .F(ext_pmem_en), .Z(pmem_addr[3]) );
  HS65_GS_AO222X4 U228 ( .A(eu_mab[4]), .B(eu_pmem_en), .C(fe_mab[4]), .D(n15), 
        .E(dbg_mem_addr[5]), .F(ext_pmem_en), .Z(pmem_addr[4]) );
  HS65_GS_AO222X4 U229 ( .A(eu_mab[5]), .B(eu_pmem_en), .C(fe_mab[5]), .D(n15), 
        .E(dbg_mem_addr[6]), .F(ext_pmem_en), .Z(pmem_addr[5]) );
  HS65_GS_AO222X4 U230 ( .A(eu_mab[6]), .B(eu_pmem_en), .C(fe_mab[6]), .D(n15), 
        .E(dbg_mem_addr[7]), .F(ext_pmem_en), .Z(pmem_addr[6]) );
  HS65_GS_AO222X4 U231 ( .A(eu_mab[7]), .B(eu_pmem_en), .C(fe_mab[7]), .D(n15), 
        .E(dbg_mem_addr[8]), .F(ext_pmem_en), .Z(pmem_addr[7]) );
  HS65_GS_AO222X4 U232 ( .A(eu_mab[0]), .B(eu_pmem_en), .C(fe_mab[0]), .D(n15), 
        .E(dbg_mem_addr[1]), .F(ext_pmem_en), .Z(pmem_addr[0]) );
  HS65_GS_OAI22X6 U233 ( .A(n106), .B(n154), .C(pmem_dout_bckup_sel), .D(n212), 
        .Z(fe_mdb_in[7]) );
  HS65_GS_OAI22X6 U234 ( .A(n108), .B(n154), .C(pmem_dout_bckup_sel), .D(n214), 
        .Z(fe_mdb_in[5]) );
  HS65_GS_OAI22X6 U235 ( .A(n101), .B(n154), .C(pmem_dout_bckup_sel), .D(n207), 
        .Z(fe_mdb_in[12]) );
  HS65_GS_OAI22X6 U236 ( .A(n109), .B(n154), .C(pmem_dout_bckup_sel), .D(n215), 
        .Z(fe_mdb_in[4]) );
  HS65_GS_OAI22X6 U237 ( .A(n107), .B(n154), .C(pmem_dout_bckup_sel), .D(n213), 
        .Z(fe_mdb_in[6]) );
  HS65_GS_NAND2AX7 U238 ( .A(eu_mdb_in_sel_0_), .B(n152), .Z(n94) );
  HS65_GS_AO22X9 U239 ( .A(ext_dmem_addr_9_), .B(n76), .C(eu_dmem_addr_9_), 
        .D(n77), .Z(dmem_addr[9]) );
  HS65_GS_AO22X9 U240 ( .A(ext_dmem_addr_10_), .B(n75), .C(eu_dmem_addr_10_), 
        .D(n78), .Z(dmem_addr[10]) );
  HS65_GS_AO22X9 U241 ( .A(ext_dmem_addr_11_), .B(n75), .C(eu_dmem_addr_11_), 
        .D(n78), .Z(dmem_addr[11]) );
  HS65_GS_AO22X9 U242 ( .A(ext_dmem_addr_12_), .B(n76), .C(eu_dmem_addr_12_), 
        .D(n78), .Z(dmem_addr[12]) );
  HS65_GS_NAND2X7 U243 ( .A(eu_mdb_in_sel_0_), .B(n152), .Z(n93) );
  HS65_GS_OAI22X6 U244 ( .A(dbg_mem_wr[0]), .B(n78), .C(eu_mb_wr[0]), .D(n76), 
        .Z(dmem_wen[0]) );
  HS65_GS_OAI22X6 U245 ( .A(dbg_mem_wr[1]), .B(n78), .C(eu_mb_wr[1]), .D(n76), 
        .Z(dmem_wen[1]) );
  HS65_GS_IVX9 U246 ( .A(eu_mab[8]), .Z(eu_dmem_addr_8_) );
  HS65_GS_AO22X9 U247 ( .A(dbg_mem_addr[2]), .B(n76), .C(eu_mab[1]), .D(n78), 
        .Z(dmem_addr[1]) );
  HS65_GS_AO22X9 U248 ( .A(dbg_mem_addr[3]), .B(n76), .C(eu_mab[2]), .D(n78), 
        .Z(dmem_addr[2]) );
  HS65_GS_AO22X9 U249 ( .A(dbg_mem_addr[4]), .B(n76), .C(eu_mab[3]), .D(n78), 
        .Z(dmem_addr[3]) );
  HS65_GS_AO22X9 U250 ( .A(dbg_mem_addr[5]), .B(n76), .C(eu_mab[4]), .D(n78), 
        .Z(dmem_addr[4]) );
  HS65_GS_AO22X9 U251 ( .A(dbg_mem_addr[6]), .B(n76), .C(eu_mab[5]), .D(n78), 
        .Z(dmem_addr[5]) );
  HS65_GS_AO22X9 U252 ( .A(dbg_mem_addr[7]), .B(n76), .C(eu_mab[6]), .D(n77), 
        .Z(dmem_addr[6]) );
  HS65_GS_AO22X9 U253 ( .A(dbg_mem_addr[8]), .B(n75), .C(eu_mab[7]), .D(n77), 
        .Z(dmem_addr[7]) );
  HS65_GS_AO22X9 U254 ( .A(dbg_mem_addr[1]), .B(n76), .C(eu_mab[0]), .D(n78), 
        .Z(dmem_addr[0]) );
  HS65_GS_IVX9 U255 ( .A(pmem_dout_bckup_sel), .Z(n154) );
  HS65_GS_IVX9 U256 ( .A(eu_mdb_out[0]), .Z(n155) );
  HS65_GS_IVX9 U257 ( .A(eu_mdb_out[1]), .Z(n158) );
  HS65_GS_IVX9 U258 ( .A(eu_mdb_out[2]), .Z(n160) );
  HS65_GS_IVX9 U259 ( .A(eu_mdb_out[3]), .Z(n162) );
  HS65_GS_IVX9 U260 ( .A(eu_mdb_out[4]), .Z(n164) );
  HS65_GS_IVX9 U261 ( .A(eu_mdb_out[5]), .Z(n166) );
  HS65_GS_IVX9 U262 ( .A(eu_mdb_out[6]), .Z(n168) );
  HS65_GS_IVX9 U263 ( .A(eu_mdb_out[7]), .Z(n170) );
  HS65_GS_IVX9 U264 ( .A(eu_mdb_out[8]), .Z(n156) );
  HS65_GS_IVX9 U265 ( .A(eu_mdb_out[9]), .Z(n159) );
  HS65_GS_IVX9 U266 ( .A(eu_mdb_out[10]), .Z(n161) );
  HS65_GS_IVX9 U267 ( .A(eu_mdb_out[11]), .Z(n163) );
  HS65_GS_IVX9 U268 ( .A(eu_mdb_out[12]), .Z(n165) );
  HS65_GS_IVX9 U269 ( .A(eu_mdb_out[13]), .Z(n167) );
  HS65_GS_IVX9 U270 ( .A(eu_mdb_out[14]), .Z(n169) );
  HS65_GS_IVX9 U271 ( .A(eu_mdb_out[15]), .Z(n171) );
  HS65_GS_IVX9 U272 ( .A(pmem_dout_bckup[0]), .Z(n135) );
  HS65_GS_IVX9 U273 ( .A(pmem_dout_bckup[1]), .Z(n114) );
  HS65_GS_IVX9 U274 ( .A(pmem_dout_bckup[2]), .Z(n111) );
  HS65_GS_IVX9 U275 ( .A(pmem_dout_bckup[3]), .Z(n110) );
  HS65_GS_IVX9 U276 ( .A(pmem_dout_bckup[4]), .Z(n109) );
  HS65_GS_IVX9 U277 ( .A(pmem_dout_bckup[5]), .Z(n108) );
  HS65_GS_IVX9 U278 ( .A(pmem_dout_bckup[7]), .Z(n106) );
  HS65_GS_IVX9 U279 ( .A(pmem_dout_bckup[8]), .Z(n105) );
  HS65_GS_IVX9 U280 ( .A(pmem_dout_bckup[9]), .Z(n104) );
  HS65_GS_IVX9 U281 ( .A(pmem_dout_bckup[10]), .Z(n103) );
  HS65_GS_IVX9 U282 ( .A(pmem_dout_bckup[11]), .Z(n102) );
  HS65_GS_IVX9 U283 ( .A(pmem_dout_bckup[13]), .Z(n100) );
  HS65_GS_IVX9 U284 ( .A(pmem_dout_bckup[14]), .Z(n99) );
  HS65_GS_IVX9 U285 ( .A(pmem_dout_bckup[15]), .Z(n98) );
  HS65_GS_IVX9 U286 ( .A(pmem_dout_bckup[6]), .Z(n107) );
  HS65_GS_IVX9 U287 ( .A(pmem_dout_bckup[12]), .Z(n101) );
  HS65_GS_NOR4ABX4 U288 ( .A(n88), .B(dbg_mem_en), .C(eu_pmem_en), .D(n90), 
        .Z(ext_pmem_en) );
  HS65_GS_BFX9 U289 ( .A(dbg_halt_cmd), .Z(cpu_halt_cmd) );
  HS65_GS_NAND2AX7 U290 ( .A(ext_mem_din_sel_0_), .B(n153), .Z(n118) );
  HS65_GS_NOR4ABX2 U291 ( .A(n90), .B(n115), .C(dbg_mem_addr[11]), .D(
        dbg_mem_addr[10]), .Z(n84) );
  HS65_GS_NOR3X4 U292 ( .A(dbg_mem_addr[12]), .B(dbg_mem_addr[9]), .C(
        dbg_mem_addr[13]), .Z(n115) );
  HS65_GS_NAND2X7 U293 ( .A(ext_mem_din_sel_0_), .B(n153), .Z(n117) );
  HS65_GS_AND3X9 U294 ( .A(dbg_mem_en), .B(n17), .C(n84), .Z(n57) );
  HS65_GS_NAND2X7 U295 ( .A(dbg_mem_wr[0]), .B(ext_pmem_en), .Z(pmem_wen[0])
         );
  HS65_GS_NAND2X7 U296 ( .A(dbg_mem_wr[1]), .B(ext_pmem_en), .Z(pmem_wen[1])
         );
  HS65_GS_NOR4ABX2 U297 ( .A(dbg_mem_en), .B(n113), .C(n55), .D(n84), .Z(n112)
         );
  HS65_GS_NOR2X6 U298 ( .A(dbg_mem_addr[14]), .B(dbg_mem_addr[15]), .Z(n90) );
  HS65_GS_IVX9 U299 ( .A(dbg_mem_dout[0]), .Z(n172) );
  HS65_GS_IVX9 U300 ( .A(dbg_mem_dout[1]), .Z(n173) );
  HS65_GS_IVX9 U301 ( .A(dbg_mem_dout[2]), .Z(n174) );
  HS65_GS_IVX9 U302 ( .A(dbg_mem_dout[3]), .Z(n175) );
  HS65_GS_IVX9 U303 ( .A(dbg_mem_dout[4]), .Z(n176) );
  HS65_GS_IVX9 U304 ( .A(dbg_mem_dout[5]), .Z(n177) );
  HS65_GS_IVX9 U305 ( .A(dbg_mem_dout[6]), .Z(n178) );
  HS65_GS_IVX9 U306 ( .A(dbg_mem_dout[7]), .Z(n179) );
  HS65_GS_IVX9 U307 ( .A(dbg_mem_dout[8]), .Z(n180) );
  HS65_GS_IVX9 U308 ( .A(dbg_mem_dout[9]), .Z(n181) );
  HS65_GS_IVX9 U309 ( .A(dbg_mem_dout[10]), .Z(n182) );
  HS65_GS_IVX9 U310 ( .A(dbg_mem_dout[11]), .Z(n183) );
  HS65_GS_IVX9 U311 ( .A(dbg_mem_dout[12]), .Z(n184) );
  HS65_GS_IVX9 U312 ( .A(dbg_mem_dout[13]), .Z(n185) );
  HS65_GS_IVX9 U313 ( .A(dbg_mem_dout[14]), .Z(n186) );
  HS65_GS_IVX9 U314 ( .A(dbg_mem_dout[15]), .Z(n187) );
  HS65_GS_BFX9 U315 ( .A(dbg_mem_dout[0]), .Z(pmem_din[0]) );
  HS65_GS_BFX9 U316 ( .A(dbg_mem_dout[1]), .Z(pmem_din[1]) );
  HS65_GS_BFX9 U317 ( .A(dbg_mem_dout[2]), .Z(pmem_din[2]) );
  HS65_GS_BFX9 U318 ( .A(dbg_mem_dout[3]), .Z(pmem_din[3]) );
  HS65_GS_BFX9 U319 ( .A(dbg_mem_dout[4]), .Z(pmem_din[4]) );
  HS65_GS_BFX9 U320 ( .A(dbg_mem_dout[5]), .Z(pmem_din[5]) );
  HS65_GS_BFX9 U321 ( .A(dbg_mem_dout[6]), .Z(pmem_din[6]) );
  HS65_GS_BFX9 U322 ( .A(dbg_mem_dout[7]), .Z(pmem_din[7]) );
  HS65_GS_BFX9 U323 ( .A(dbg_mem_dout[8]), .Z(pmem_din[8]) );
  HS65_GS_BFX9 U324 ( .A(dbg_mem_dout[9]), .Z(pmem_din[9]) );
  HS65_GS_BFX9 U325 ( .A(dbg_mem_dout[10]), .Z(pmem_din[10]) );
  HS65_GS_BFX9 U326 ( .A(dbg_mem_dout[11]), .Z(pmem_din[11]) );
  HS65_GS_BFX9 U327 ( .A(dbg_mem_dout[12]), .Z(pmem_din[12]) );
  HS65_GS_BFX9 U328 ( .A(dbg_mem_dout[13]), .Z(pmem_din[13]) );
  HS65_GS_BFX9 U329 ( .A(dbg_mem_dout[14]), .Z(pmem_din[14]) );
  HS65_GS_BFX9 U330 ( .A(dbg_mem_dout[15]), .Z(pmem_din[15]) );
  HS65_GSS_XNOR2X3 U331 ( .A(dbg_mem_addr[15]), .B(dbg_mem_addr[14]), .Z(
        ext_pmem_addr_14_) );
  HS65_GSS_XNOR2X3 U332 ( .A(fe_mab[14]), .B(fe_mab[13]), .Z(fe_pmem_addr_14_)
         );
  HS65_GSS_XNOR2X3 U333 ( .A(eu_mab[14]), .B(eu_mab[13]), .Z(eu_pmem_addr_14_)
         );
  HS65_GSS_XNOR2X3 U334 ( .A(eu_mab[12]), .B(sub_230_carry[12]), .Z(
        eu_dmem_addr_12_) );
  HS65_GS_OR2X4 U335 ( .A(sub_230_carry[11]), .B(eu_mab[11]), .Z(
        sub_230_carry[12]) );
  HS65_GSS_XNOR2X3 U336 ( .A(eu_mab[11]), .B(sub_230_carry[11]), .Z(
        eu_dmem_addr_11_) );
  HS65_GS_OR2X4 U337 ( .A(sub_230_carry[10]), .B(eu_mab[10]), .Z(
        sub_230_carry[11]) );
  HS65_GSS_XNOR2X3 U338 ( .A(eu_mab[10]), .B(sub_230_carry[10]), .Z(
        eu_dmem_addr_10_) );
  HS65_GS_OR2X4 U339 ( .A(eu_mab[8]), .B(eu_mab[9]), .Z(sub_230_carry[10]) );
  HS65_GSS_XNOR2X3 U340 ( .A(eu_mab[9]), .B(eu_mab[8]), .Z(eu_dmem_addr_9_) );
  HS65_GSS_XNOR2X3 U341 ( .A(dbg_mem_addr[13]), .B(sub_239_carry[12]), .Z(
        ext_dmem_addr_12_) );
  HS65_GS_OR2X4 U342 ( .A(sub_239_carry[11]), .B(dbg_mem_addr[12]), .Z(
        sub_239_carry[12]) );
  HS65_GSS_XNOR2X3 U343 ( .A(dbg_mem_addr[12]), .B(sub_239_carry[11]), .Z(
        ext_dmem_addr_11_) );
  HS65_GS_OR2X4 U344 ( .A(sub_239_carry[10]), .B(dbg_mem_addr[11]), .Z(
        sub_239_carry[11]) );
  HS65_GSS_XNOR2X3 U345 ( .A(dbg_mem_addr[11]), .B(sub_239_carry[10]), .Z(
        ext_dmem_addr_10_) );
  HS65_GS_OR2X4 U346 ( .A(dbg_mem_addr[9]), .B(dbg_mem_addr[10]), .Z(
        sub_239_carry[10]) );
  HS65_GSS_XNOR2X3 U347 ( .A(dbg_mem_addr[10]), .B(dbg_mem_addr[9]), .Z(
        ext_dmem_addr_9_) );
  HS65_GS_CB4I6X4 U348 ( .A(eu_mab[9]), .B(eu_mab[8]), .C(eu_mab[10]), .D(
        eu_mab[11]), .Z(n83) );
  HS65_GS_AOI112X1 U349 ( .A(eu_mab[12]), .B(n83), .C(eu_mab[14]), .D(
        eu_mab[13]), .Z(N11) );
  HS65_GS_CB4I6X4 U350 ( .A(dbg_mem_addr[10]), .B(dbg_mem_addr[9]), .C(
        dbg_mem_addr[11]), .D(dbg_mem_addr[12]), .Z(n86) );
endmodule


module omsp_sfr ( cpu_id, nmi_pnd, nmi_wkup, per_dout, wdtie, wdtifg_sw_clr, 
        wdtifg_sw_set, cpu_nr_inst, cpu_nr_total, mclk, nmi, nmi_acc, per_addr, 
        per_din, per_en, per_we, puc_rst, scan_mode, wdtifg, wdtnmies );
  output [31:0] cpu_id;
  output [15:0] per_dout;
  input [7:0] cpu_nr_inst;
  input [7:0] cpu_nr_total;
  input [13:0] per_addr;
  input [15:0] per_din;
  input [1:0] per_we;
  input mclk, nmi, nmi_acc, per_en, puc_rst, scan_mode, wdtifg, wdtnmies;
  output nmi_pnd, nmi_wkup, wdtie, wdtifg_sw_clr, wdtifg_sw_set;
  wire   n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n85, n86;

  HS65_GS_IVX9 U3 ( .A(1'b1), .Z(wdtifg_sw_set) );
  HS65_GS_IVX9 U5 ( .A(1'b1), .Z(wdtifg_sw_clr) );
  HS65_GS_IVX9 U7 ( .A(1'b1), .Z(wdtie) );
  HS65_GS_IVX9 U9 ( .A(1'b1), .Z(nmi_wkup) );
  HS65_GS_IVX9 U11 ( .A(1'b1), .Z(nmi_pnd) );
  HS65_GS_IVX9 U13 ( .A(1'b0), .Z(cpu_id[0]) );
  HS65_GS_IVX9 U15 ( .A(1'b0), .Z(cpu_id[1]) );
  HS65_GS_IVX9 U17 ( .A(1'b1), .Z(cpu_id[2]) );
  HS65_GS_IVX9 U19 ( .A(1'b0), .Z(cpu_id[3]) );
  HS65_GS_IVX9 U21 ( .A(1'b1), .Z(cpu_id[4]) );
  HS65_GS_IVX9 U23 ( .A(1'b1), .Z(cpu_id[5]) );
  HS65_GS_IVX9 U25 ( .A(1'b1), .Z(cpu_id[6]) );
  HS65_GS_IVX9 U27 ( .A(1'b1), .Z(cpu_id[7]) );
  HS65_GS_IVX9 U29 ( .A(1'b1), .Z(cpu_id[8]) );
  HS65_GS_IVX9 U31 ( .A(1'b0), .Z(cpu_id[9]) );
  HS65_GS_IVX9 U33 ( .A(1'b1), .Z(cpu_id[10]) );
  HS65_GS_IVX9 U35 ( .A(1'b1), .Z(cpu_id[11]) );
  HS65_GS_IVX9 U37 ( .A(1'b1), .Z(cpu_id[12]) );
  HS65_GS_IVX9 U39 ( .A(1'b1), .Z(cpu_id[13]) );
  HS65_GS_IVX9 U41 ( .A(1'b1), .Z(cpu_id[14]) );
  HS65_GS_IVX9 U43 ( .A(1'b1), .Z(cpu_id[15]) );
  HS65_GS_IVX9 U45 ( .A(1'b0), .Z(cpu_id[16]) );
  HS65_GS_IVX9 U47 ( .A(1'b1), .Z(cpu_id[17]) );
  HS65_GS_IVX9 U49 ( .A(1'b1), .Z(cpu_id[18]) );
  HS65_GS_IVX9 U51 ( .A(1'b1), .Z(cpu_id[19]) );
  HS65_GS_IVX9 U53 ( .A(1'b1), .Z(cpu_id[20]) );
  HS65_GS_IVX9 U55 ( .A(1'b0), .Z(cpu_id[21]) );
  HS65_GS_IVX9 U57 ( .A(1'b1), .Z(cpu_id[22]) );
  HS65_GS_IVX9 U59 ( .A(1'b0), .Z(cpu_id[23]) );
  HS65_GS_IVX9 U61 ( .A(1'b1), .Z(cpu_id[24]) );
  HS65_GS_IVX9 U63 ( .A(1'b1), .Z(cpu_id[25]) );
  HS65_GS_IVX9 U65 ( .A(1'b1), .Z(cpu_id[26]) );
  HS65_GS_IVX9 U67 ( .A(1'b1), .Z(cpu_id[27]) );
  HS65_GS_IVX9 U69 ( .A(1'b1), .Z(cpu_id[28]) );
  HS65_GS_IVX9 U71 ( .A(1'b1), .Z(cpu_id[29]) );
  HS65_GS_IVX9 U73 ( .A(1'b0), .Z(cpu_id[30]) );
  HS65_GS_IVX9 U75 ( .A(1'b0), .Z(cpu_id[31]) );
  HS65_GS_IVX9 U77 ( .A(n5), .Z(n85) );
  HS65_GS_IVX9 U78 ( .A(n4), .Z(n86) );
  HS65_GS_NOR2AX3 U79 ( .A(n8), .B(per_addr[2]), .Z(n7) );
  HS65_GS_NAND3X5 U80 ( .A(per_addr[1]), .B(n7), .C(per_addr[0]), .Z(n5) );
  HS65_GS_NAND3AX6 U81 ( .A(per_addr[0]), .B(n7), .C(per_addr[1]), .Z(n4) );
  HS65_GS_NOR4ABX4 U82 ( .A(n8), .B(per_addr[2]), .C(per_addr[0]), .D(
        per_addr[1]), .Z(n3) );
  HS65_GS_AND3X9 U83 ( .A(n9), .B(n10), .C(n11), .Z(n8) );
  HS65_GS_NOR4X4 U84 ( .A(per_we[1]), .B(per_we[0]), .C(per_addr[9]), .D(
        per_addr[8]), .Z(n10) );
  HS65_GS_NOR3X4 U85 ( .A(per_addr[5]), .B(per_addr[7]), .C(per_addr[6]), .Z(
        n9) );
  HS65_GS_NOR4ABX2 U86 ( .A(per_en), .B(n12), .C(per_addr[11]), .D(
        per_addr[10]), .Z(n11) );
  HS65_GS_NAND3X5 U87 ( .A(n5), .B(n4), .C(n6), .Z(per_dout[0]) );
  HS65_GS_AOI32X5 U88 ( .A(per_addr[0]), .B(n7), .C(wdtifg), .D(cpu_nr_inst[0]), .E(n3), .Z(n6) );
  HS65_GS_AO12X9 U89 ( .A(cpu_nr_inst[5]), .B(n3), .C(n85), .Z(per_dout[5]) );
  HS65_GS_AO12X9 U90 ( .A(cpu_nr_inst[7]), .B(n3), .C(n85), .Z(per_dout[7]) );
  HS65_GS_AO12X9 U91 ( .A(cpu_nr_total[6]), .B(n3), .C(n85), .Z(per_dout[14])
         );
  HS65_GS_AO12X9 U92 ( .A(cpu_nr_total[7]), .B(n3), .C(n85), .Z(per_dout[15])
         );
  HS65_GS_AO12X9 U93 ( .A(cpu_nr_inst[1]), .B(n3), .C(n86), .Z(per_dout[1]) );
  HS65_GS_AO12X9 U94 ( .A(cpu_nr_inst[3]), .B(n3), .C(n86), .Z(per_dout[3]) );
  HS65_GS_AO12X9 U95 ( .A(cpu_nr_total[1]), .B(n3), .C(n86), .Z(per_dout[9])
         );
  HS65_GS_NOR4X4 U96 ( .A(per_addr[4]), .B(per_addr[3]), .C(per_addr[13]), .D(
        per_addr[12]), .Z(n12) );
  HS65_GS_AND2X4 U97 ( .A(cpu_nr_inst[2]), .B(n3), .Z(per_dout[2]) );
  HS65_GS_AND2X4 U98 ( .A(cpu_nr_inst[4]), .B(n3), .Z(per_dout[4]) );
  HS65_GS_AND2X4 U99 ( .A(cpu_nr_inst[6]), .B(n3), .Z(per_dout[6]) );
  HS65_GS_AND2X4 U100 ( .A(cpu_nr_total[0]), .B(n3), .Z(per_dout[8]) );
  HS65_GS_AND2X4 U101 ( .A(cpu_nr_total[2]), .B(n3), .Z(per_dout[10]) );
  HS65_GS_AND2X4 U102 ( .A(cpu_nr_total[3]), .B(n3), .Z(per_dout[11]) );
  HS65_GS_AND2X4 U103 ( .A(cpu_nr_total[4]), .B(n3), .Z(per_dout[12]) );
  HS65_GS_AND2X4 U104 ( .A(cpu_nr_total[5]), .B(n3), .Z(per_dout[13]) );
endmodule


module omsp_multiplier_DW01_add_0 ( A, B, CI, SUM, CO );
  input [32:0] A;
  input [32:0] B;
  output [32:0] SUM;
  input CI;
  output CO;
  wire   n1;
  wire   [31:2] carry;

  HS65_GS_FA1X4 U1_6 ( .A0(A[6]), .B0(B[6]), .CI(carry[6]), .CO(carry[7]), 
        .S0(SUM[6]) );
  HS65_GS_FA1X4 U1_30 ( .A0(A[30]), .B0(B[30]), .CI(carry[30]), .CO(carry[31]), 
        .S0(SUM[30]) );
  HS65_GS_FA1X4 U1_29 ( .A0(A[29]), .B0(B[29]), .CI(carry[29]), .CO(carry[30]), 
        .S0(SUM[29]) );
  HS65_GS_FA1X4 U1_28 ( .A0(A[28]), .B0(B[28]), .CI(carry[28]), .CO(carry[29]), 
        .S0(SUM[28]) );
  HS65_GS_FA1X4 U1_27 ( .A0(A[27]), .B0(B[27]), .CI(carry[27]), .CO(carry[28]), 
        .S0(SUM[27]) );
  HS65_GS_FA1X4 U1_26 ( .A0(A[26]), .B0(B[26]), .CI(carry[26]), .CO(carry[27]), 
        .S0(SUM[26]) );
  HS65_GS_FA1X4 U1_25 ( .A0(A[25]), .B0(B[25]), .CI(carry[25]), .CO(carry[26]), 
        .S0(SUM[25]) );
  HS65_GS_FA1X4 U1_24 ( .A0(A[24]), .B0(B[24]), .CI(carry[24]), .CO(carry[25]), 
        .S0(SUM[24]) );
  HS65_GS_FA1X4 U1_23 ( .A0(A[23]), .B0(B[23]), .CI(carry[23]), .CO(carry[24]), 
        .S0(SUM[23]) );
  HS65_GS_FA1X4 U1_21 ( .A0(A[21]), .B0(B[21]), .CI(carry[21]), .CO(carry[22]), 
        .S0(SUM[21]) );
  HS65_GS_FA1X4 U1_20 ( .A0(A[20]), .B0(B[20]), .CI(carry[20]), .CO(carry[21]), 
        .S0(SUM[20]) );
  HS65_GS_FA1X4 U1_19 ( .A0(A[19]), .B0(B[19]), .CI(carry[19]), .CO(carry[20]), 
        .S0(SUM[19]) );
  HS65_GS_FA1X4 U1_18 ( .A0(A[18]), .B0(B[18]), .CI(carry[18]), .CO(carry[19]), 
        .S0(SUM[18]) );
  HS65_GS_FA1X4 U1_15 ( .A0(A[15]), .B0(B[15]), .CI(carry[15]), .CO(carry[16]), 
        .S0(SUM[15]) );
  HS65_GS_FA1X4 U1_14 ( .A0(A[14]), .B0(B[14]), .CI(carry[14]), .CO(carry[15]), 
        .S0(SUM[14]) );
  HS65_GS_FA1X4 U1_13 ( .A0(A[13]), .B0(B[13]), .CI(carry[13]), .CO(carry[14]), 
        .S0(SUM[13]) );
  HS65_GS_FA1X4 U1_11 ( .A0(A[11]), .B0(B[11]), .CI(carry[11]), .CO(carry[12]), 
        .S0(SUM[11]) );
  HS65_GS_FA1X4 U1_10 ( .A0(A[10]), .B0(B[10]), .CI(carry[10]), .CO(carry[11]), 
        .S0(SUM[10]) );
  HS65_GS_FA1X4 U1_7 ( .A0(A[7]), .B0(B[7]), .CI(carry[7]), .CO(carry[8]), 
        .S0(SUM[7]) );
  HS65_GS_FA1X4 U1_5 ( .A0(A[5]), .B0(B[5]), .CI(carry[5]), .CO(carry[6]), 
        .S0(SUM[5]) );
  HS65_GS_FA1X4 U1_4 ( .A0(A[4]), .B0(B[4]), .CI(carry[4]), .CO(carry[5]), 
        .S0(SUM[4]) );
  HS65_GS_FA1X4 U1_3 ( .A0(A[3]), .B0(B[3]), .CI(carry[3]), .CO(carry[4]), 
        .S0(SUM[3]) );
  HS65_GS_FA1X4 U1_2 ( .A0(A[2]), .B0(B[2]), .CI(carry[2]), .CO(carry[3]), 
        .S0(SUM[2]) );
  HS65_GS_FA1X4 U1_1 ( .A0(A[1]), .B0(B[1]), .CI(n1), .CO(carry[2]), .S0(
        SUM[1]) );
  HS65_GS_FA1X4 U1_16 ( .A0(A[16]), .B0(B[16]), .CI(carry[16]), .CO(carry[17]), 
        .S0(SUM[16]) );
  HS65_GS_FA1X4 U1_31 ( .A0(A[31]), .B0(B[31]), .CI(carry[31]), .CO(SUM[32]), 
        .S0(SUM[31]) );
  HS65_GS_FA1X4 U1_12 ( .A0(A[12]), .B0(B[12]), .CI(carry[12]), .CO(carry[13]), 
        .S0(SUM[12]) );
  HS65_GS_FA1X4 U1_9 ( .A0(A[9]), .B0(B[9]), .CI(carry[9]), .CO(carry[10]), 
        .S0(SUM[9]) );
  HS65_GS_FA1X4 U1_17 ( .A0(A[17]), .B0(B[17]), .CI(carry[17]), .CO(carry[18]), 
        .S0(SUM[17]) );
  HS65_GS_FA1X4 U1_22 ( .A0(A[22]), .B0(B[22]), .CI(carry[22]), .CO(carry[23]), 
        .S0(SUM[22]) );
  HS65_GS_FA1X4 U1_8 ( .A0(A[8]), .B0(B[8]), .CI(carry[8]), .CO(carry[9]), 
        .S0(SUM[8]) );
  HS65_GS_AND2X4 U1 ( .A(A[0]), .B(B[0]), .Z(n1) );
  HS65_GSS_XOR2X6 U2 ( .A(A[0]), .B(B[0]), .Z(SUM[0]) );
endmodule


module omsp_multiplier_DW01_add_1 ( A, B, CI, SUM, CO );
  input [23:0] A;
  input [23:0] B;
  output [23:0] SUM;
  input CI;
  output CO;
  wire   n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19, n20, n21,
         n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33, n34, n35,
         n36, n37, n38, n39, n40, n41;

  HS65_GS_IVX9 U2 ( .A(n23), .Z(n8) );
  HS65_GS_IVX9 U3 ( .A(n31), .Z(n9) );
  HS65_GS_BFX9 U4 ( .A(A[0]), .Z(SUM[0]) );
  HS65_GS_BFX9 U5 ( .A(A[1]), .Z(SUM[1]) );
  HS65_GS_BFX9 U6 ( .A(A[2]), .Z(SUM[2]) );
  HS65_GS_BFX9 U7 ( .A(A[3]), .Z(SUM[3]) );
  HS65_GS_BFX9 U8 ( .A(A[4]), .Z(SUM[4]) );
  HS65_GS_BFX9 U9 ( .A(A[5]), .Z(SUM[5]) );
  HS65_GS_BFX9 U10 ( .A(A[6]), .Z(SUM[6]) );
  HS65_GS_IVX9 U11 ( .A(A[12]), .Z(n10) );
  HS65_GS_IVX9 U12 ( .A(A[8]), .Z(n11) );
  HS65_GSS_XOR2X3 U13 ( .A(A[9]), .B(n12), .Z(SUM[9]) );
  HS65_GS_NOR2X2 U14 ( .A(n13), .B(n11), .Z(n12) );
  HS65_GSS_XNOR2X3 U15 ( .A(A[8]), .B(n13), .Z(SUM[8]) );
  HS65_GS_OA12X4 U16 ( .A(B[7]), .B(A[7]), .C(n13), .Z(SUM[7]) );
  HS65_GSS_XOR3X2 U17 ( .A(B[21]), .B(A[21]), .C(n14), .Z(SUM[21]) );
  HS65_GS_AO12X4 U18 ( .A(n15), .B(A[20]), .C(n16), .Z(n14) );
  HS65_GS_OA12X4 U19 ( .A(A[20]), .B(n15), .C(B[20]), .Z(n16) );
  HS65_GSS_XOR3X2 U20 ( .A(n15), .B(B[20]), .C(A[20]), .Z(SUM[20]) );
  HS65_GS_OAI12X2 U21 ( .A(n17), .B(n18), .C(n19), .Z(n15) );
  HS65_GSS_XOR2X3 U22 ( .A(n20), .B(n18), .Z(SUM[19]) );
  HS65_GS_AOI12X2 U23 ( .A(n8), .B(n21), .C(n22), .Z(n18) );
  HS65_GS_NAND2AX4 U24 ( .A(n17), .B(n19), .Z(n20) );
  HS65_GS_NAND2X2 U25 ( .A(B[19]), .B(A[19]), .Z(n19) );
  HS65_GS_NOR2X2 U26 ( .A(B[19]), .B(A[19]), .Z(n17) );
  HS65_GSS_XOR2X3 U27 ( .A(n21), .B(n24), .Z(SUM[18]) );
  HS65_GS_NOR2X2 U28 ( .A(n22), .B(n23), .Z(n24) );
  HS65_GS_NOR2X2 U29 ( .A(B[18]), .B(A[18]), .Z(n23) );
  HS65_GS_AND2X4 U30 ( .A(B[18]), .B(A[18]), .Z(n22) );
  HS65_GS_OAI12X2 U31 ( .A(n25), .B(n26), .C(n27), .Z(n21) );
  HS65_GSS_XOR2X3 U32 ( .A(n28), .B(n26), .Z(SUM[17]) );
  HS65_GS_AOI12X2 U33 ( .A(n9), .B(n29), .C(n30), .Z(n26) );
  HS65_GS_NAND2AX4 U34 ( .A(n25), .B(n27), .Z(n28) );
  HS65_GS_NAND2X2 U35 ( .A(B[17]), .B(A[17]), .Z(n27) );
  HS65_GS_NOR2X2 U36 ( .A(B[17]), .B(A[17]), .Z(n25) );
  HS65_GSS_XOR2X3 U37 ( .A(n29), .B(n32), .Z(SUM[16]) );
  HS65_GS_NOR2X2 U38 ( .A(n30), .B(n31), .Z(n32) );
  HS65_GS_NOR2X2 U39 ( .A(B[16]), .B(A[16]), .Z(n31) );
  HS65_GS_AND2X4 U40 ( .A(B[16]), .B(A[16]), .Z(n30) );
  HS65_GS_OAI12X2 U41 ( .A(n33), .B(n34), .C(n35), .Z(n29) );
  HS65_GSS_XOR2X3 U42 ( .A(n34), .B(n36), .Z(SUM[15]) );
  HS65_GS_NAND2AX4 U43 ( .A(n33), .B(n35), .Z(n36) );
  HS65_GS_NAND2X2 U44 ( .A(B[15]), .B(A[15]), .Z(n35) );
  HS65_GS_NOR2X2 U45 ( .A(B[15]), .B(A[15]), .Z(n33) );
  HS65_GS_NAND2X2 U46 ( .A(A[14]), .B(n37), .Z(n34) );
  HS65_GSS_XOR2X3 U47 ( .A(A[14]), .B(n37), .Z(SUM[14]) );
  HS65_GS_NOR3AX2 U48 ( .A(A[13]), .B(n10), .C(n38), .Z(n37) );
  HS65_GSS_XOR2X3 U49 ( .A(A[13]), .B(n39), .Z(SUM[13]) );
  HS65_GS_NOR2X2 U50 ( .A(n38), .B(n10), .Z(n39) );
  HS65_GSS_XNOR2X3 U51 ( .A(A[12]), .B(n38), .Z(SUM[12]) );
  HS65_GS_NAND3X2 U52 ( .A(A[10]), .B(n40), .C(A[11]), .Z(n38) );
  HS65_GSS_XNOR2X3 U53 ( .A(A[11]), .B(n41), .Z(SUM[11]) );
  HS65_GS_NAND2X2 U54 ( .A(A[10]), .B(n40), .Z(n41) );
  HS65_GSS_XOR2X3 U55 ( .A(A[10]), .B(n40), .Z(SUM[10]) );
  HS65_GS_NOR3AX2 U56 ( .A(A[9]), .B(n11), .C(n13), .Z(n40) );
  HS65_GS_NAND2X2 U57 ( .A(B[7]), .B(A[7]), .Z(n13) );
endmodule


module omsp_multiplier_DW02_mult_0 ( A, B, TC, PRODUCT );
  input [16:0] A;
  input [8:0] B;
  output [25:0] PRODUCT;
  input TC;
  wire   ab_16__8_, ab_16__7_, ab_16__6_, ab_16__5_, ab_16__4_, ab_16__3_,
         ab_16__2_, ab_16__1_, ab_16__0_, ab_15__8_, ab_15__7_, ab_15__6_,
         ab_15__5_, ab_15__4_, ab_15__3_, ab_15__2_, ab_15__1_, ab_15__0_,
         ab_14__8_, ab_14__7_, ab_14__6_, ab_14__5_, ab_14__4_, ab_14__3_,
         ab_14__2_, ab_14__1_, ab_14__0_, ab_13__8_, ab_13__7_, ab_13__6_,
         ab_13__5_, ab_13__4_, ab_13__3_, ab_13__2_, ab_13__1_, ab_13__0_,
         ab_12__8_, ab_12__7_, ab_12__6_, ab_12__5_, ab_12__4_, ab_12__3_,
         ab_12__2_, ab_12__1_, ab_12__0_, ab_11__8_, ab_11__7_, ab_11__6_,
         ab_11__5_, ab_11__4_, ab_11__3_, ab_11__2_, ab_11__1_, ab_11__0_,
         ab_10__8_, ab_10__7_, ab_10__6_, ab_10__5_, ab_10__4_, ab_10__3_,
         ab_10__2_, ab_10__1_, ab_10__0_, ab_9__8_, ab_9__7_, ab_9__6_,
         ab_9__5_, ab_9__4_, ab_9__3_, ab_9__2_, ab_9__1_, ab_9__0_, ab_8__8_,
         ab_8__7_, ab_8__6_, ab_8__5_, ab_8__4_, ab_8__3_, ab_8__2_, ab_8__1_,
         ab_8__0_, ab_7__8_, ab_7__7_, ab_7__6_, ab_7__5_, ab_7__4_, ab_7__3_,
         ab_7__2_, ab_7__1_, ab_7__0_, ab_6__8_, ab_6__7_, ab_6__6_, ab_6__5_,
         ab_6__4_, ab_6__3_, ab_6__2_, ab_6__1_, ab_6__0_, ab_5__8_, ab_5__7_,
         ab_5__6_, ab_5__5_, ab_5__4_, ab_5__3_, ab_5__2_, ab_5__1_, ab_5__0_,
         ab_4__8_, ab_4__7_, ab_4__6_, ab_4__5_, ab_4__4_, ab_4__3_, ab_4__2_,
         ab_4__1_, ab_4__0_, ab_3__8_, ab_3__7_, ab_3__6_, ab_3__5_, ab_3__4_,
         ab_3__3_, ab_3__2_, ab_3__1_, ab_3__0_, ab_2__8_, ab_2__7_, ab_2__6_,
         ab_2__5_, ab_2__4_, ab_2__3_, ab_2__2_, ab_2__1_, ab_2__0_, ab_1__8_,
         ab_1__7_, ab_1__6_, ab_1__5_, ab_1__4_, ab_1__3_, ab_1__2_, ab_1__1_,
         ab_1__0_, ab_0__8_, ab_0__7_, ab_0__6_, ab_0__5_, ab_0__4_, ab_0__3_,
         ab_0__2_, ab_0__1_, CARRYB_16__8_, CARRYB_16__7_, CARRYB_16__6_,
         CARRYB_16__5_, CARRYB_16__4_, CARRYB_16__3_, CARRYB_16__2_,
         CARRYB_16__1_, CARRYB_16__0_, CARRYB_15__7_, CARRYB_15__6_,
         CARRYB_15__5_, CARRYB_15__4_, CARRYB_15__3_, CARRYB_15__2_,
         CARRYB_15__1_, CARRYB_15__0_, CARRYB_14__7_, CARRYB_14__6_,
         CARRYB_14__5_, CARRYB_14__4_, CARRYB_14__3_, CARRYB_14__2_,
         CARRYB_14__1_, CARRYB_14__0_, CARRYB_13__7_, CARRYB_13__6_,
         CARRYB_13__5_, CARRYB_13__4_, CARRYB_13__3_, CARRYB_13__2_,
         CARRYB_13__1_, CARRYB_13__0_, CARRYB_12__7_, CARRYB_12__6_,
         CARRYB_12__5_, CARRYB_12__4_, CARRYB_12__3_, CARRYB_12__2_,
         CARRYB_12__1_, CARRYB_12__0_, CARRYB_11__7_, CARRYB_11__6_,
         CARRYB_11__5_, CARRYB_11__4_, CARRYB_11__3_, CARRYB_11__2_,
         CARRYB_11__1_, CARRYB_11__0_, CARRYB_10__7_, CARRYB_10__6_,
         CARRYB_10__5_, CARRYB_10__4_, CARRYB_10__3_, CARRYB_10__2_,
         CARRYB_10__1_, CARRYB_10__0_, CARRYB_9__7_, CARRYB_9__6_,
         CARRYB_9__5_, CARRYB_9__4_, CARRYB_9__3_, CARRYB_9__2_, CARRYB_9__1_,
         CARRYB_9__0_, CARRYB_8__7_, CARRYB_8__6_, CARRYB_8__5_, CARRYB_8__4_,
         CARRYB_8__3_, CARRYB_8__2_, CARRYB_8__1_, CARRYB_8__0_, CARRYB_7__7_,
         CARRYB_7__6_, CARRYB_7__5_, CARRYB_7__4_, CARRYB_7__3_, CARRYB_7__2_,
         CARRYB_7__1_, CARRYB_7__0_, CARRYB_6__7_, CARRYB_6__6_, CARRYB_6__5_,
         CARRYB_6__4_, CARRYB_6__3_, CARRYB_6__2_, CARRYB_6__1_, CARRYB_6__0_,
         CARRYB_5__7_, CARRYB_5__6_, CARRYB_5__5_, CARRYB_5__4_, CARRYB_5__3_,
         CARRYB_5__2_, CARRYB_5__1_, CARRYB_5__0_, CARRYB_4__7_, CARRYB_4__6_,
         CARRYB_4__5_, CARRYB_4__4_, CARRYB_4__3_, CARRYB_4__2_, CARRYB_4__1_,
         CARRYB_4__0_, CARRYB_3__7_, CARRYB_3__6_, CARRYB_3__5_, CARRYB_3__4_,
         CARRYB_3__3_, CARRYB_3__2_, CARRYB_3__1_, CARRYB_3__0_, CARRYB_2__7_,
         CARRYB_2__6_, CARRYB_2__5_, CARRYB_2__4_, CARRYB_2__3_, CARRYB_2__2_,
         CARRYB_2__1_, CARRYB_2__0_, SUMB_16__8_, SUMB_16__7_, SUMB_16__6_,
         SUMB_16__5_, SUMB_16__4_, SUMB_16__3_, SUMB_16__2_, SUMB_16__1_,
         SUMB_16__0_, SUMB_15__7_, SUMB_15__6_, SUMB_15__5_, SUMB_15__4_,
         SUMB_15__3_, SUMB_15__2_, SUMB_15__1_, SUMB_14__7_, SUMB_14__6_,
         SUMB_14__5_, SUMB_14__4_, SUMB_14__3_, SUMB_14__2_, SUMB_14__1_,
         SUMB_13__7_, SUMB_13__6_, SUMB_13__5_, SUMB_13__4_, SUMB_13__3_,
         SUMB_13__2_, SUMB_13__1_, SUMB_12__7_, SUMB_12__6_, SUMB_12__5_,
         SUMB_12__4_, SUMB_12__3_, SUMB_12__2_, SUMB_12__1_, SUMB_11__7_,
         SUMB_11__6_, SUMB_11__5_, SUMB_11__4_, SUMB_11__3_, SUMB_11__2_,
         SUMB_11__1_, SUMB_10__7_, SUMB_10__6_, SUMB_10__5_, SUMB_10__4_,
         SUMB_10__3_, SUMB_10__2_, SUMB_10__1_, SUMB_9__7_, SUMB_9__6_,
         SUMB_9__5_, SUMB_9__4_, SUMB_9__3_, SUMB_9__2_, SUMB_9__1_,
         SUMB_8__7_, SUMB_8__6_, SUMB_8__5_, SUMB_8__4_, SUMB_8__3_,
         SUMB_8__2_, SUMB_8__1_, SUMB_7__7_, SUMB_7__6_, SUMB_7__5_,
         SUMB_7__4_, SUMB_7__3_, SUMB_7__2_, SUMB_7__1_, SUMB_6__7_,
         SUMB_6__6_, SUMB_6__5_, SUMB_6__4_, SUMB_6__3_, SUMB_6__2_,
         SUMB_6__1_, SUMB_5__7_, SUMB_5__6_, SUMB_5__5_, SUMB_5__4_,
         SUMB_5__3_, SUMB_5__2_, SUMB_5__1_, SUMB_4__7_, SUMB_4__6_,
         SUMB_4__5_, SUMB_4__4_, SUMB_4__3_, SUMB_4__2_, SUMB_4__1_,
         SUMB_3__7_, SUMB_3__6_, SUMB_3__5_, SUMB_3__4_, SUMB_3__3_,
         SUMB_3__2_, SUMB_3__1_, SUMB_2__7_, SUMB_2__6_, SUMB_2__5_,
         SUMB_2__4_, SUMB_2__3_, SUMB_2__2_, SUMB_2__1_, SUMB_1__7_,
         SUMB_1__6_, SUMB_1__5_, SUMB_1__4_, SUMB_1__3_, SUMB_1__2_,
         SUMB_1__1_, PROD1_8_, A1_23_, A1_22_, A1_21_, A1_20_, A1_19_, A1_18_,
         A1_17_, A1_16_, A1_15_, A1_14_, A1_13_, A1_12_, A1_11_, A1_10_, A1_9_,
         A1_8_, A1_7_, A1_6_, A1_5_, A1_4_, A1_3_, A1_2_, A1_1_, A1_0_, n3, n4,
         n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19,
         n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33,
         n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44, n45, n46,
         SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2;

  omsp_multiplier_DW01_add_1 FS_1 ( .A({A1_23_, A1_22_, A1_21_, A1_20_, A1_19_, 
        A1_18_, A1_17_, A1_16_, A1_15_, A1_14_, A1_13_, A1_12_, A1_11_, A1_10_, 
        A1_9_, A1_8_, A1_7_, A1_6_, A1_5_, A1_4_, A1_3_, A1_2_, A1_1_, A1_0_}), 
        .B({n4, n3, n13, n14, n15, n18, n16, n17, n20, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, n19, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), 
        .CI(1'b0), .SUM({SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2, 
        PRODUCT[23:2]}) );
  HS65_GS_FA1X4 S5_7 ( .A0(ab_16__7_), .B0(CARRYB_15__7_), .CI(ab_15__8_), 
        .CO(CARRYB_16__7_), .S0(SUMB_16__7_) );
  HS65_GS_FA1X4 S3_15_7 ( .A0(ab_15__7_), .B0(CARRYB_14__7_), .CI(ab_14__8_), 
        .CO(CARRYB_15__7_), .S0(SUMB_15__7_) );
  HS65_GS_FA1X4 S3_14_7 ( .A0(ab_14__7_), .B0(CARRYB_13__7_), .CI(ab_13__8_), 
        .CO(CARRYB_14__7_), .S0(SUMB_14__7_) );
  HS65_GS_FA1X4 S3_13_7 ( .A0(ab_13__7_), .B0(CARRYB_12__7_), .CI(ab_12__8_), 
        .CO(CARRYB_13__7_), .S0(SUMB_13__7_) );
  HS65_GS_FA1X4 S3_12_7 ( .A0(ab_12__7_), .B0(CARRYB_11__7_), .CI(ab_11__8_), 
        .CO(CARRYB_12__7_), .S0(SUMB_12__7_) );
  HS65_GS_FA1X4 S3_11_7 ( .A0(ab_11__7_), .B0(CARRYB_10__7_), .CI(ab_10__8_), 
        .CO(CARRYB_11__7_), .S0(SUMB_11__7_) );
  HS65_GS_FA1X4 S3_10_7 ( .A0(ab_10__7_), .B0(CARRYB_9__7_), .CI(ab_9__8_), 
        .CO(CARRYB_10__7_), .S0(SUMB_10__7_) );
  HS65_GS_FA1X4 S3_9_7 ( .A0(ab_9__7_), .B0(CARRYB_8__7_), .CI(ab_8__8_), .CO(
        CARRYB_9__7_), .S0(SUMB_9__7_) );
  HS65_GS_FA1X4 S3_8_7 ( .A0(ab_8__7_), .B0(CARRYB_7__7_), .CI(ab_7__8_), .CO(
        CARRYB_8__7_), .S0(SUMB_8__7_) );
  HS65_GS_FA1X4 S3_7_7 ( .A0(ab_7__7_), .B0(CARRYB_6__7_), .CI(ab_6__8_), .CO(
        CARRYB_7__7_), .S0(SUMB_7__7_) );
  HS65_GS_FA1X4 S3_6_7 ( .A0(ab_6__7_), .B0(CARRYB_5__7_), .CI(ab_5__8_), .CO(
        CARRYB_6__7_), .S0(SUMB_6__7_) );
  HS65_GS_FA1X4 S3_5_7 ( .A0(ab_5__7_), .B0(CARRYB_4__7_), .CI(ab_4__8_), .CO(
        CARRYB_5__7_), .S0(SUMB_5__7_) );
  HS65_GS_FA1X4 S3_4_7 ( .A0(ab_4__7_), .B0(CARRYB_3__7_), .CI(ab_3__8_), .CO(
        CARRYB_4__7_), .S0(SUMB_4__7_) );
  HS65_GS_FA1X4 S3_3_7 ( .A0(ab_3__7_), .B0(CARRYB_2__7_), .CI(ab_2__8_), .CO(
        CARRYB_3__7_), .S0(SUMB_3__7_) );
  HS65_GS_FA1X4 S3_2_7 ( .A0(ab_2__7_), .B0(n5), .CI(ab_1__8_), .CO(
        CARRYB_2__7_), .S0(SUMB_2__7_) );
  HS65_GS_FA1X4 S2_14_1 ( .A0(ab_14__1_), .B0(CARRYB_13__1_), .CI(SUMB_13__2_), 
        .CO(CARRYB_14__1_), .S0(SUMB_14__1_) );
  HS65_GS_FA1X4 S2_14_2 ( .A0(ab_14__2_), .B0(CARRYB_13__2_), .CI(SUMB_13__3_), 
        .CO(CARRYB_14__2_), .S0(SUMB_14__2_) );
  HS65_GS_FA1X4 S2_12_2 ( .A0(ab_12__2_), .B0(CARRYB_11__2_), .CI(SUMB_11__3_), 
        .CO(CARRYB_12__2_), .S0(SUMB_12__2_) );
  HS65_GS_FA1X4 S2_10_2 ( .A0(ab_10__2_), .B0(CARRYB_9__2_), .CI(SUMB_9__3_), 
        .CO(CARRYB_10__2_), .S0(SUMB_10__2_) );
  HS65_GS_FA1X4 S2_15_3 ( .A0(ab_15__3_), .B0(CARRYB_14__3_), .CI(SUMB_14__4_), 
        .CO(CARRYB_15__3_), .S0(SUMB_15__3_) );
  HS65_GS_FA1X4 S2_14_3 ( .A0(ab_14__3_), .B0(CARRYB_13__3_), .CI(SUMB_13__4_), 
        .CO(CARRYB_14__3_), .S0(SUMB_14__3_) );
  HS65_GS_FA1X4 S2_12_3 ( .A0(ab_12__3_), .B0(CARRYB_11__3_), .CI(SUMB_11__4_), 
        .CO(CARRYB_12__3_), .S0(SUMB_12__3_) );
  HS65_GS_FA1X4 S2_10_3 ( .A0(ab_10__3_), .B0(CARRYB_9__3_), .CI(SUMB_9__4_), 
        .CO(CARRYB_10__3_), .S0(SUMB_10__3_) );
  HS65_GS_FA1X4 S2_14_4 ( .A0(ab_14__4_), .B0(CARRYB_13__4_), .CI(SUMB_13__5_), 
        .CO(CARRYB_14__4_), .S0(SUMB_14__4_) );
  HS65_GS_FA1X4 S2_13_4 ( .A0(ab_13__4_), .B0(CARRYB_12__4_), .CI(SUMB_12__5_), 
        .CO(CARRYB_13__4_), .S0(SUMB_13__4_) );
  HS65_GS_FA1X4 S2_12_4 ( .A0(ab_12__4_), .B0(CARRYB_11__4_), .CI(SUMB_11__5_), 
        .CO(CARRYB_12__4_), .S0(SUMB_12__4_) );
  HS65_GS_FA1X4 S2_10_4 ( .A0(ab_10__4_), .B0(CARRYB_9__4_), .CI(SUMB_9__5_), 
        .CO(CARRYB_10__4_), .S0(SUMB_10__4_) );
  HS65_GS_FA1X4 S2_8_4 ( .A0(ab_8__4_), .B0(CARRYB_7__4_), .CI(SUMB_7__5_), 
        .CO(CARRYB_8__4_), .S0(SUMB_8__4_) );
  HS65_GS_FA1X4 S2_15_5 ( .A0(ab_15__5_), .B0(CARRYB_14__5_), .CI(SUMB_14__6_), 
        .CO(CARRYB_15__5_), .S0(SUMB_15__5_) );
  HS65_GS_FA1X4 S2_14_5 ( .A0(ab_14__5_), .B0(CARRYB_13__5_), .CI(SUMB_13__6_), 
        .CO(CARRYB_14__5_), .S0(SUMB_14__5_) );
  HS65_GS_FA1X4 S2_13_5 ( .A0(ab_13__5_), .B0(CARRYB_12__5_), .CI(SUMB_12__6_), 
        .CO(CARRYB_13__5_), .S0(SUMB_13__5_) );
  HS65_GS_FA1X4 S2_12_5 ( .A0(ab_12__5_), .B0(CARRYB_11__5_), .CI(SUMB_11__6_), 
        .CO(CARRYB_12__5_), .S0(SUMB_12__5_) );
  HS65_GS_FA1X4 S2_11_5 ( .A0(ab_11__5_), .B0(CARRYB_10__5_), .CI(SUMB_10__6_), 
        .CO(CARRYB_11__5_), .S0(SUMB_11__5_) );
  HS65_GS_FA1X4 S2_10_5 ( .A0(ab_10__5_), .B0(CARRYB_9__5_), .CI(SUMB_9__6_), 
        .CO(CARRYB_10__5_), .S0(SUMB_10__5_) );
  HS65_GS_FA1X4 S2_8_5 ( .A0(ab_8__5_), .B0(CARRYB_7__5_), .CI(SUMB_7__6_), 
        .CO(CARRYB_8__5_), .S0(SUMB_8__5_) );
  HS65_GS_FA1X4 S2_6_5 ( .A0(ab_6__5_), .B0(CARRYB_5__5_), .CI(SUMB_5__6_), 
        .CO(CARRYB_6__5_), .S0(SUMB_6__5_) );
  HS65_GS_FA1X4 S2_4_5 ( .A0(ab_4__5_), .B0(CARRYB_3__5_), .CI(SUMB_3__6_), 
        .CO(CARRYB_4__5_), .S0(SUMB_4__5_) );
  HS65_GS_FA1X4 S2_15_4 ( .A0(ab_15__4_), .B0(CARRYB_14__4_), .CI(SUMB_14__5_), 
        .CO(CARRYB_15__4_), .S0(SUMB_15__4_) );
  HS65_GS_FA1X4 S2_15_2 ( .A0(ab_15__2_), .B0(CARRYB_14__2_), .CI(SUMB_14__3_), 
        .CO(CARRYB_15__2_), .S0(SUMB_15__2_) );
  HS65_GS_FA1X4 S2_15_1 ( .A0(ab_15__1_), .B0(CARRYB_14__1_), .CI(SUMB_14__2_), 
        .CO(CARRYB_15__1_), .S0(SUMB_15__1_) );
  HS65_GS_FA1X4 S1_15_0 ( .A0(ab_15__0_), .B0(CARRYB_14__0_), .CI(SUMB_14__1_), 
        .CO(CARRYB_15__0_), .S0(A1_13_) );
  HS65_GS_FA1X4 S1_14_0 ( .A0(ab_14__0_), .B0(CARRYB_13__0_), .CI(SUMB_13__1_), 
        .CO(CARRYB_14__0_), .S0(A1_12_) );
  HS65_GS_FA1X4 S2_13_3 ( .A0(ab_13__3_), .B0(CARRYB_12__3_), .CI(SUMB_12__4_), 
        .CO(CARRYB_13__3_), .S0(SUMB_13__3_) );
  HS65_GS_FA1X4 S2_13_2 ( .A0(ab_13__2_), .B0(CARRYB_12__2_), .CI(SUMB_12__3_), 
        .CO(CARRYB_13__2_), .S0(SUMB_13__2_) );
  HS65_GS_FA1X4 S2_13_1 ( .A0(ab_13__1_), .B0(CARRYB_12__1_), .CI(SUMB_12__2_), 
        .CO(CARRYB_13__1_), .S0(SUMB_13__1_) );
  HS65_GS_FA1X4 S1_13_0 ( .A0(ab_13__0_), .B0(CARRYB_12__0_), .CI(SUMB_12__1_), 
        .CO(CARRYB_13__0_), .S0(A1_11_) );
  HS65_GS_FA1X4 S2_12_1 ( .A0(ab_12__1_), .B0(CARRYB_11__1_), .CI(SUMB_11__2_), 
        .CO(CARRYB_12__1_), .S0(SUMB_12__1_) );
  HS65_GS_FA1X4 S1_12_0 ( .A0(ab_12__0_), .B0(CARRYB_11__0_), .CI(SUMB_11__1_), 
        .CO(CARRYB_12__0_), .S0(A1_10_) );
  HS65_GS_FA1X4 S2_11_4 ( .A0(ab_11__4_), .B0(CARRYB_10__4_), .CI(SUMB_10__5_), 
        .CO(CARRYB_11__4_), .S0(SUMB_11__4_) );
  HS65_GS_FA1X4 S2_11_3 ( .A0(ab_11__3_), .B0(CARRYB_10__3_), .CI(SUMB_10__4_), 
        .CO(CARRYB_11__3_), .S0(SUMB_11__3_) );
  HS65_GS_FA1X4 S2_11_2 ( .A0(ab_11__2_), .B0(CARRYB_10__2_), .CI(SUMB_10__3_), 
        .CO(CARRYB_11__2_), .S0(SUMB_11__2_) );
  HS65_GS_FA1X4 S2_11_1 ( .A0(ab_11__1_), .B0(CARRYB_10__1_), .CI(SUMB_10__2_), 
        .CO(CARRYB_11__1_), .S0(SUMB_11__1_) );
  HS65_GS_FA1X4 S1_11_0 ( .A0(ab_11__0_), .B0(CARRYB_10__0_), .CI(SUMB_10__1_), 
        .CO(CARRYB_11__0_), .S0(A1_9_) );
  HS65_GS_FA1X4 S2_10_1 ( .A0(ab_10__1_), .B0(CARRYB_9__1_), .CI(SUMB_9__2_), 
        .CO(CARRYB_10__1_), .S0(SUMB_10__1_) );
  HS65_GS_FA1X4 S1_10_0 ( .A0(ab_10__0_), .B0(CARRYB_9__0_), .CI(SUMB_9__1_), 
        .CO(CARRYB_10__0_), .S0(A1_8_) );
  HS65_GS_FA1X4 S2_8_3 ( .A0(ab_8__3_), .B0(CARRYB_7__3_), .CI(SUMB_7__4_), 
        .CO(CARRYB_8__3_), .S0(SUMB_8__3_) );
  HS65_GS_FA1X4 S2_8_2 ( .A0(ab_8__2_), .B0(CARRYB_7__2_), .CI(SUMB_7__3_), 
        .CO(CARRYB_8__2_), .S0(SUMB_8__2_) );
  HS65_GS_FA1X4 S2_8_1 ( .A0(ab_8__1_), .B0(CARRYB_7__1_), .CI(SUMB_7__2_), 
        .CO(CARRYB_8__1_), .S0(SUMB_8__1_) );
  HS65_GS_FA1X4 S1_8_0 ( .A0(ab_8__0_), .B0(CARRYB_7__0_), .CI(SUMB_7__1_), 
        .CO(CARRYB_8__0_), .S0(PROD1_8_) );
  HS65_GS_FA1X4 S2_7_6 ( .A0(ab_7__6_), .B0(CARRYB_6__6_), .CI(SUMB_6__7_), 
        .CO(CARRYB_7__6_), .S0(SUMB_7__6_) );
  HS65_GS_FA1X4 S2_7_5 ( .A0(ab_7__5_), .B0(CARRYB_6__5_), .CI(SUMB_6__6_), 
        .CO(CARRYB_7__5_), .S0(SUMB_7__5_) );
  HS65_GS_FA1X4 S2_7_4 ( .A0(ab_7__4_), .B0(CARRYB_6__4_), .CI(SUMB_6__5_), 
        .CO(CARRYB_7__4_), .S0(SUMB_7__4_) );
  HS65_GS_FA1X4 S2_7_3 ( .A0(ab_7__3_), .B0(CARRYB_6__3_), .CI(SUMB_6__4_), 
        .CO(CARRYB_7__3_), .S0(SUMB_7__3_) );
  HS65_GS_FA1X4 S2_7_2 ( .A0(ab_7__2_), .B0(CARRYB_6__2_), .CI(SUMB_6__3_), 
        .CO(CARRYB_7__2_), .S0(SUMB_7__2_) );
  HS65_GS_FA1X4 S2_7_1 ( .A0(ab_7__1_), .B0(CARRYB_6__1_), .CI(SUMB_6__2_), 
        .CO(CARRYB_7__1_), .S0(SUMB_7__1_) );
  HS65_GS_FA1X4 S1_7_0 ( .A0(ab_7__0_), .B0(CARRYB_6__0_), .CI(SUMB_6__1_), 
        .CO(CARRYB_7__0_), .S0(A1_5_) );
  HS65_GS_FA1X4 S2_6_4 ( .A0(ab_6__4_), .B0(CARRYB_5__4_), .CI(SUMB_5__5_), 
        .CO(CARRYB_6__4_), .S0(SUMB_6__4_) );
  HS65_GS_FA1X4 S2_6_3 ( .A0(ab_6__3_), .B0(CARRYB_5__3_), .CI(SUMB_5__4_), 
        .CO(CARRYB_6__3_), .S0(SUMB_6__3_) );
  HS65_GS_FA1X4 S2_6_2 ( .A0(ab_6__2_), .B0(CARRYB_5__2_), .CI(SUMB_5__3_), 
        .CO(CARRYB_6__2_), .S0(SUMB_6__2_) );
  HS65_GS_FA1X4 S2_6_1 ( .A0(ab_6__1_), .B0(CARRYB_5__1_), .CI(SUMB_5__2_), 
        .CO(CARRYB_6__1_), .S0(SUMB_6__1_) );
  HS65_GS_FA1X4 S1_6_0 ( .A0(ab_6__0_), .B0(CARRYB_5__0_), .CI(SUMB_5__1_), 
        .CO(CARRYB_6__0_), .S0(A1_4_) );
  HS65_GS_FA1X4 S2_5_6 ( .A0(ab_5__6_), .B0(CARRYB_4__6_), .CI(SUMB_4__7_), 
        .CO(CARRYB_5__6_), .S0(SUMB_5__6_) );
  HS65_GS_FA1X4 S2_5_5 ( .A0(ab_5__5_), .B0(CARRYB_4__5_), .CI(SUMB_4__6_), 
        .CO(CARRYB_5__5_), .S0(SUMB_5__5_) );
  HS65_GS_FA1X4 S2_5_4 ( .A0(ab_5__4_), .B0(CARRYB_4__4_), .CI(SUMB_4__5_), 
        .CO(CARRYB_5__4_), .S0(SUMB_5__4_) );
  HS65_GS_FA1X4 S2_5_3 ( .A0(ab_5__3_), .B0(CARRYB_4__3_), .CI(SUMB_4__4_), 
        .CO(CARRYB_5__3_), .S0(SUMB_5__3_) );
  HS65_GS_FA1X4 S2_5_2 ( .A0(ab_5__2_), .B0(CARRYB_4__2_), .CI(SUMB_4__3_), 
        .CO(CARRYB_5__2_), .S0(SUMB_5__2_) );
  HS65_GS_FA1X4 S2_5_1 ( .A0(ab_5__1_), .B0(CARRYB_4__1_), .CI(SUMB_4__2_), 
        .CO(CARRYB_5__1_), .S0(SUMB_5__1_) );
  HS65_GS_FA1X4 S1_5_0 ( .A0(ab_5__0_), .B0(CARRYB_4__0_), .CI(SUMB_4__1_), 
        .CO(CARRYB_5__0_), .S0(A1_3_) );
  HS65_GS_FA1X4 S2_4_6 ( .A0(ab_4__6_), .B0(CARRYB_3__6_), .CI(SUMB_3__7_), 
        .CO(CARRYB_4__6_), .S0(SUMB_4__6_) );
  HS65_GS_FA1X4 S2_4_4 ( .A0(ab_4__4_), .B0(CARRYB_3__4_), .CI(SUMB_3__5_), 
        .CO(CARRYB_4__4_), .S0(SUMB_4__4_) );
  HS65_GS_FA1X4 S2_4_3 ( .A0(ab_4__3_), .B0(CARRYB_3__3_), .CI(SUMB_3__4_), 
        .CO(CARRYB_4__3_), .S0(SUMB_4__3_) );
  HS65_GS_FA1X4 S2_4_2 ( .A0(ab_4__2_), .B0(CARRYB_3__2_), .CI(SUMB_3__3_), 
        .CO(CARRYB_4__2_), .S0(SUMB_4__2_) );
  HS65_GS_FA1X4 S2_4_1 ( .A0(ab_4__1_), .B0(CARRYB_3__1_), .CI(SUMB_3__2_), 
        .CO(CARRYB_4__1_), .S0(SUMB_4__1_) );
  HS65_GS_FA1X4 S1_4_0 ( .A0(ab_4__0_), .B0(CARRYB_3__0_), .CI(SUMB_3__1_), 
        .CO(CARRYB_4__0_), .S0(A1_2_) );
  HS65_GS_FA1X4 S2_3_6 ( .A0(ab_3__6_), .B0(CARRYB_2__6_), .CI(SUMB_2__7_), 
        .CO(CARRYB_3__6_), .S0(SUMB_3__6_) );
  HS65_GS_FA1X4 S2_3_5 ( .A0(ab_3__5_), .B0(CARRYB_2__5_), .CI(SUMB_2__6_), 
        .CO(CARRYB_3__5_), .S0(SUMB_3__5_) );
  HS65_GS_FA1X4 S2_3_4 ( .A0(ab_3__4_), .B0(CARRYB_2__4_), .CI(SUMB_2__5_), 
        .CO(CARRYB_3__4_), .S0(SUMB_3__4_) );
  HS65_GS_FA1X4 S2_3_3 ( .A0(ab_3__3_), .B0(CARRYB_2__3_), .CI(SUMB_2__4_), 
        .CO(CARRYB_3__3_), .S0(SUMB_3__3_) );
  HS65_GS_FA1X4 S2_3_2 ( .A0(ab_3__2_), .B0(CARRYB_2__2_), .CI(SUMB_2__3_), 
        .CO(CARRYB_3__2_), .S0(SUMB_3__2_) );
  HS65_GS_FA1X4 S2_3_1 ( .A0(ab_3__1_), .B0(CARRYB_2__1_), .CI(SUMB_2__2_), 
        .CO(CARRYB_3__1_), .S0(SUMB_3__1_) );
  HS65_GS_FA1X4 S1_3_0 ( .A0(ab_3__0_), .B0(CARRYB_2__0_), .CI(SUMB_2__1_), 
        .CO(CARRYB_3__0_), .S0(A1_1_) );
  HS65_GS_FA1X4 S2_2_6 ( .A0(ab_2__6_), .B0(n9), .CI(SUMB_1__7_), .CO(
        CARRYB_2__6_), .S0(SUMB_2__6_) );
  HS65_GS_FA1X4 S2_2_4 ( .A0(ab_2__4_), .B0(n8), .CI(SUMB_1__5_), .CO(
        CARRYB_2__4_), .S0(SUMB_2__4_) );
  HS65_GS_FA1X4 S2_2_3 ( .A0(ab_2__3_), .B0(n7), .CI(SUMB_1__4_), .CO(
        CARRYB_2__3_), .S0(SUMB_2__3_) );
  HS65_GS_FA1X4 S2_2_2 ( .A0(ab_2__2_), .B0(n12), .CI(SUMB_1__3_), .CO(
        CARRYB_2__2_), .S0(SUMB_2__2_) );
  HS65_GS_FA1X4 S2_2_1 ( .A0(ab_2__1_), .B0(n11), .CI(SUMB_1__2_), .CO(
        CARRYB_2__1_), .S0(SUMB_2__1_) );
  HS65_GS_FA1X4 S1_2_0 ( .A0(ab_2__0_), .B0(n10), .CI(SUMB_1__1_), .CO(
        CARRYB_2__0_), .S0(A1_0_) );
  HS65_GS_FA1X4 S2_9_5 ( .A0(ab_9__5_), .B0(CARRYB_8__5_), .CI(SUMB_8__6_), 
        .CO(CARRYB_9__5_), .S0(SUMB_9__5_) );
  HS65_GS_FA1X4 S2_9_4 ( .A0(ab_9__4_), .B0(CARRYB_8__4_), .CI(SUMB_8__5_), 
        .CO(CARRYB_9__4_), .S0(SUMB_9__4_) );
  HS65_GS_FA1X4 S2_9_3 ( .A0(ab_9__3_), .B0(CARRYB_8__3_), .CI(SUMB_8__4_), 
        .CO(CARRYB_9__3_), .S0(SUMB_9__3_) );
  HS65_GS_FA1X4 S2_9_2 ( .A0(ab_9__2_), .B0(CARRYB_8__2_), .CI(SUMB_8__3_), 
        .CO(CARRYB_9__2_), .S0(SUMB_9__2_) );
  HS65_GS_FA1X4 S2_9_1 ( .A0(ab_9__1_), .B0(CARRYB_8__1_), .CI(SUMB_8__2_), 
        .CO(CARRYB_9__1_), .S0(SUMB_9__1_) );
  HS65_GS_FA1X4 S1_9_0 ( .A0(ab_9__0_), .B0(CARRYB_8__0_), .CI(SUMB_8__1_), 
        .CO(CARRYB_9__0_), .S0(A1_7_) );
  HS65_GS_FA1X4 S2_15_6 ( .A0(ab_15__6_), .B0(CARRYB_14__6_), .CI(SUMB_14__7_), 
        .CO(CARRYB_15__6_), .S0(SUMB_15__6_) );
  HS65_GS_FA1X4 S2_14_6 ( .A0(ab_14__6_), .B0(CARRYB_13__6_), .CI(SUMB_13__7_), 
        .CO(CARRYB_14__6_), .S0(SUMB_14__6_) );
  HS65_GS_FA1X4 S2_13_6 ( .A0(ab_13__6_), .B0(CARRYB_12__6_), .CI(SUMB_12__7_), 
        .CO(CARRYB_13__6_), .S0(SUMB_13__6_) );
  HS65_GS_FA1X4 S2_12_6 ( .A0(ab_12__6_), .B0(CARRYB_11__6_), .CI(SUMB_11__7_), 
        .CO(CARRYB_12__6_), .S0(SUMB_12__6_) );
  HS65_GS_FA1X4 S2_11_6 ( .A0(ab_11__6_), .B0(CARRYB_10__6_), .CI(SUMB_10__7_), 
        .CO(CARRYB_11__6_), .S0(SUMB_11__6_) );
  HS65_GS_FA1X4 S2_10_6 ( .A0(ab_10__6_), .B0(CARRYB_9__6_), .CI(SUMB_9__7_), 
        .CO(CARRYB_10__6_), .S0(SUMB_10__6_) );
  HS65_GS_FA1X4 S2_9_6 ( .A0(ab_9__6_), .B0(CARRYB_8__6_), .CI(SUMB_8__7_), 
        .CO(CARRYB_9__6_), .S0(SUMB_9__6_) );
  HS65_GS_FA1X4 S2_8_6 ( .A0(ab_8__6_), .B0(CARRYB_7__6_), .CI(SUMB_7__7_), 
        .CO(CARRYB_8__6_), .S0(SUMB_8__6_) );
  HS65_GS_FA1X4 S2_6_6 ( .A0(ab_6__6_), .B0(CARRYB_5__6_), .CI(SUMB_5__7_), 
        .CO(CARRYB_6__6_), .S0(SUMB_6__6_) );
  HS65_GS_FA1X4 S4_6 ( .A0(ab_16__6_), .B0(CARRYB_15__6_), .CI(SUMB_15__7_), 
        .CO(CARRYB_16__6_), .S0(SUMB_16__6_) );
  HS65_GS_FA1X4 S2_2_5 ( .A0(ab_2__5_), .B0(n6), .CI(SUMB_1__6_), .CO(
        CARRYB_2__5_), .S0(SUMB_2__5_) );
  HS65_GS_FA1X4 S4_2 ( .A0(ab_16__2_), .B0(CARRYB_15__2_), .CI(SUMB_15__3_), 
        .CO(CARRYB_16__2_), .S0(SUMB_16__2_) );
  HS65_GS_FA1X4 S4_1 ( .A0(ab_16__1_), .B0(CARRYB_15__1_), .CI(SUMB_15__2_), 
        .CO(CARRYB_16__1_), .S0(SUMB_16__1_) );
  HS65_GS_FA1X4 S4_0 ( .A0(ab_16__0_), .B0(CARRYB_15__0_), .CI(SUMB_15__1_), 
        .CO(CARRYB_16__0_), .S0(SUMB_16__0_) );
  HS65_GS_FA1X4 S4_3 ( .A0(ab_16__3_), .B0(CARRYB_15__3_), .CI(SUMB_15__4_), 
        .CO(CARRYB_16__3_), .S0(SUMB_16__3_) );
  HS65_GS_FA1X4 S4_4 ( .A0(ab_16__4_), .B0(CARRYB_15__4_), .CI(SUMB_15__5_), 
        .CO(CARRYB_16__4_), .S0(SUMB_16__4_) );
  HS65_GS_FA1X4 S4_5 ( .A0(ab_16__5_), .B0(CARRYB_15__5_), .CI(SUMB_15__6_), 
        .CO(CARRYB_16__5_), .S0(SUMB_16__5_) );
  HS65_GS_FA1X4 S14_8 ( .A0(n22), .B0(n38), .CI(ab_16__8_), .CO(CARRYB_16__8_), 
        .S0(SUMB_16__8_) );
  HS65_GS_AND2X4 U2 ( .A(SUMB_16__7_), .B(CARRYB_16__6_), .Z(n3) );
  HS65_GS_AND2X4 U3 ( .A(SUMB_16__8_), .B(CARRYB_16__7_), .Z(n4) );
  HS65_GS_AND2X4 U4 ( .A(ab_1__7_), .B(ab_0__8_), .Z(n5) );
  HS65_GS_AND2X4 U5 ( .A(ab_1__5_), .B(ab_0__6_), .Z(n6) );
  HS65_GS_AND2X4 U6 ( .A(ab_1__3_), .B(ab_0__4_), .Z(n7) );
  HS65_GS_AND2X4 U7 ( .A(ab_1__4_), .B(ab_0__5_), .Z(n8) );
  HS65_GS_AND2X4 U8 ( .A(ab_1__6_), .B(ab_0__7_), .Z(n9) );
  HS65_GS_AND2X4 U9 ( .A(ab_1__0_), .B(ab_0__1_), .Z(n10) );
  HS65_GS_AND2X4 U10 ( .A(ab_1__1_), .B(ab_0__2_), .Z(n11) );
  HS65_GS_AND2X4 U11 ( .A(ab_1__2_), .B(ab_0__3_), .Z(n12) );
  HS65_GS_AND2X4 U12 ( .A(SUMB_16__6_), .B(CARRYB_16__5_), .Z(n13) );
  HS65_GSS_XOR2X6 U13 ( .A(SUMB_16__5_), .B(CARRYB_16__4_), .Z(A1_19_) );
  HS65_GSS_XOR2X6 U14 ( .A(SUMB_16__4_), .B(CARRYB_16__3_), .Z(A1_18_) );
  HS65_GSS_XOR2X6 U15 ( .A(SUMB_16__3_), .B(CARRYB_16__2_), .Z(A1_17_) );
  HS65_GSS_XOR2X6 U16 ( .A(SUMB_16__1_), .B(CARRYB_16__0_), .Z(A1_15_) );
  HS65_GSS_XOR2X6 U17 ( .A(SUMB_16__2_), .B(CARRYB_16__1_), .Z(A1_16_) );
  HS65_GS_AND2X4 U18 ( .A(SUMB_16__5_), .B(CARRYB_16__4_), .Z(n14) );
  HS65_GS_AND2X4 U19 ( .A(SUMB_16__4_), .B(CARRYB_16__3_), .Z(n15) );
  HS65_GS_AND2X4 U20 ( .A(SUMB_16__2_), .B(CARRYB_16__1_), .Z(n16) );
  HS65_GS_AND2X4 U21 ( .A(SUMB_16__1_), .B(CARRYB_16__0_), .Z(n17) );
  HS65_GS_AND2X4 U22 ( .A(SUMB_16__3_), .B(CARRYB_16__2_), .Z(n18) );
  HS65_GSS_XOR2X6 U23 ( .A(ab_0__1_), .B(ab_1__0_), .Z(PRODUCT[1]) );
  HS65_GS_IVX9 U24 ( .A(B[8]), .Z(n38) );
  HS65_GSS_XOR2X6 U25 ( .A(SUMB_16__0_), .B(A[16]), .Z(A1_14_) );
  HS65_GSS_XOR2X6 U26 ( .A(SUMB_16__6_), .B(CARRYB_16__5_), .Z(A1_20_) );
  HS65_GS_IVX9 U27 ( .A(B[0]), .Z(n46) );
  HS65_GS_IVX9 U28 ( .A(B[1]), .Z(n45) );
  HS65_GS_IVX9 U29 ( .A(B[2]), .Z(n44) );
  HS65_GS_IVX9 U30 ( .A(B[3]), .Z(n43) );
  HS65_GS_IVX9 U31 ( .A(B[4]), .Z(n42) );
  HS65_GS_IVX9 U32 ( .A(B[7]), .Z(n39) );
  HS65_GS_IVX9 U33 ( .A(B[6]), .Z(n40) );
  HS65_GS_IVX9 U34 ( .A(B[5]), .Z(n41) );
  HS65_GSS_XOR2X6 U35 ( .A(PROD1_8_), .B(B[8]), .Z(A1_6_) );
  HS65_GS_AND2X4 U36 ( .A(PROD1_8_), .B(B[8]), .Z(n19) );
  HS65_GS_AND2X4 U37 ( .A(SUMB_16__0_), .B(A[16]), .Z(n20) );
  HS65_GS_IVX9 U38 ( .A(A[16]), .Z(n22) );
  HS65_GSS_XOR2X6 U39 ( .A(SUMB_16__7_), .B(CARRYB_16__6_), .Z(A1_21_) );
  HS65_GSS_XOR2X6 U40 ( .A(ab_0__7_), .B(ab_1__6_), .Z(SUMB_1__6_) );
  HS65_GSS_XOR2X6 U41 ( .A(ab_0__2_), .B(ab_1__1_), .Z(SUMB_1__1_) );
  HS65_GSS_XOR2X6 U42 ( .A(ab_0__3_), .B(ab_1__2_), .Z(SUMB_1__2_) );
  HS65_GSS_XOR2X6 U43 ( .A(ab_0__4_), .B(ab_1__3_), .Z(SUMB_1__3_) );
  HS65_GSS_XOR2X6 U44 ( .A(ab_0__5_), .B(ab_1__4_), .Z(SUMB_1__4_) );
  HS65_GSS_XOR2X6 U45 ( .A(ab_0__6_), .B(ab_1__5_), .Z(SUMB_1__5_) );
  HS65_GSS_XOR2X6 U46 ( .A(ab_0__8_), .B(ab_1__7_), .Z(SUMB_1__7_) );
  HS65_GSS_XOR2X6 U47 ( .A(SUMB_16__8_), .B(CARRYB_16__7_), .Z(A1_22_) );
  HS65_GS_IVX9 U48 ( .A(A[0]), .Z(n37) );
  HS65_GS_IVX9 U49 ( .A(A[1]), .Z(n36) );
  HS65_GS_IVX9 U50 ( .A(A[2]), .Z(n35) );
  HS65_GS_IVX9 U51 ( .A(A[3]), .Z(n34) );
  HS65_GS_IVX9 U52 ( .A(A[4]), .Z(n33) );
  HS65_GS_IVX9 U53 ( .A(A[8]), .Z(n29) );
  HS65_GS_IVX9 U54 ( .A(A[10]), .Z(n27) );
  HS65_GS_IVX9 U55 ( .A(A[11]), .Z(n26) );
  HS65_GS_IVX9 U56 ( .A(A[12]), .Z(n25) );
  HS65_GS_IVX9 U57 ( .A(A[13]), .Z(n24) );
  HS65_GS_IVX9 U58 ( .A(A[14]), .Z(n23) );
  HS65_GS_IVX9 U59 ( .A(A[15]), .Z(n21) );
  HS65_GS_IVX9 U60 ( .A(A[5]), .Z(n32) );
  HS65_GS_IVX9 U61 ( .A(A[6]), .Z(n31) );
  HS65_GS_IVX9 U62 ( .A(A[7]), .Z(n30) );
  HS65_GS_IVX9 U63 ( .A(A[9]), .Z(n28) );
  HS65_GS_NOR2X2 U64 ( .A(A[9]), .B(n38), .Z(ab_9__8_) );
  HS65_GS_NOR2X2 U65 ( .A(n28), .B(n39), .Z(ab_9__7_) );
  HS65_GS_NOR2X2 U66 ( .A(n28), .B(n40), .Z(ab_9__6_) );
  HS65_GS_NOR2X2 U67 ( .A(n28), .B(n41), .Z(ab_9__5_) );
  HS65_GS_NOR2X2 U68 ( .A(n28), .B(n42), .Z(ab_9__4_) );
  HS65_GS_NOR2X2 U69 ( .A(n28), .B(n43), .Z(ab_9__3_) );
  HS65_GS_NOR2X2 U70 ( .A(n28), .B(n44), .Z(ab_9__2_) );
  HS65_GS_NOR2X2 U71 ( .A(n28), .B(n45), .Z(ab_9__1_) );
  HS65_GS_NOR2X2 U72 ( .A(n28), .B(n46), .Z(ab_9__0_) );
  HS65_GS_NOR2X2 U73 ( .A(A[8]), .B(n38), .Z(ab_8__8_) );
  HS65_GS_NOR2X2 U74 ( .A(n39), .B(n29), .Z(ab_8__7_) );
  HS65_GS_NOR2X2 U75 ( .A(n40), .B(n29), .Z(ab_8__6_) );
  HS65_GS_NOR2X2 U76 ( .A(n41), .B(n29), .Z(ab_8__5_) );
  HS65_GS_NOR2X2 U77 ( .A(n42), .B(n29), .Z(ab_8__4_) );
  HS65_GS_NOR2X2 U78 ( .A(n43), .B(n29), .Z(ab_8__3_) );
  HS65_GS_NOR2X2 U79 ( .A(n44), .B(n29), .Z(ab_8__2_) );
  HS65_GS_NOR2X2 U80 ( .A(n45), .B(n29), .Z(ab_8__1_) );
  HS65_GS_NOR2X2 U81 ( .A(n46), .B(n29), .Z(ab_8__0_) );
  HS65_GS_NOR2X2 U82 ( .A(A[7]), .B(n38), .Z(ab_7__8_) );
  HS65_GS_NOR2X2 U83 ( .A(n39), .B(n30), .Z(ab_7__7_) );
  HS65_GS_NOR2X2 U84 ( .A(n40), .B(n30), .Z(ab_7__6_) );
  HS65_GS_NOR2X2 U85 ( .A(n41), .B(n30), .Z(ab_7__5_) );
  HS65_GS_NOR2X2 U86 ( .A(n42), .B(n30), .Z(ab_7__4_) );
  HS65_GS_NOR2X2 U87 ( .A(n43), .B(n30), .Z(ab_7__3_) );
  HS65_GS_NOR2X2 U88 ( .A(n44), .B(n30), .Z(ab_7__2_) );
  HS65_GS_NOR2X2 U89 ( .A(n45), .B(n30), .Z(ab_7__1_) );
  HS65_GS_NOR2X2 U90 ( .A(n46), .B(n30), .Z(ab_7__0_) );
  HS65_GS_NOR2X2 U91 ( .A(A[6]), .B(n38), .Z(ab_6__8_) );
  HS65_GS_NOR2X2 U92 ( .A(n39), .B(n31), .Z(ab_6__7_) );
  HS65_GS_NOR2X2 U93 ( .A(n40), .B(n31), .Z(ab_6__6_) );
  HS65_GS_NOR2X2 U94 ( .A(n41), .B(n31), .Z(ab_6__5_) );
  HS65_GS_NOR2X2 U95 ( .A(n42), .B(n31), .Z(ab_6__4_) );
  HS65_GS_NOR2X2 U96 ( .A(n43), .B(n31), .Z(ab_6__3_) );
  HS65_GS_NOR2X2 U97 ( .A(n44), .B(n31), .Z(ab_6__2_) );
  HS65_GS_NOR2X2 U98 ( .A(n45), .B(n31), .Z(ab_6__1_) );
  HS65_GS_NOR2X2 U99 ( .A(n46), .B(n31), .Z(ab_6__0_) );
  HS65_GS_NOR2X2 U100 ( .A(A[5]), .B(n38), .Z(ab_5__8_) );
  HS65_GS_NOR2X2 U101 ( .A(n39), .B(n32), .Z(ab_5__7_) );
  HS65_GS_NOR2X2 U102 ( .A(n40), .B(n32), .Z(ab_5__6_) );
  HS65_GS_NOR2X2 U103 ( .A(n41), .B(n32), .Z(ab_5__5_) );
  HS65_GS_NOR2X2 U104 ( .A(n42), .B(n32), .Z(ab_5__4_) );
  HS65_GS_NOR2X2 U105 ( .A(n43), .B(n32), .Z(ab_5__3_) );
  HS65_GS_NOR2X2 U106 ( .A(n44), .B(n32), .Z(ab_5__2_) );
  HS65_GS_NOR2X2 U107 ( .A(n45), .B(n32), .Z(ab_5__1_) );
  HS65_GS_NOR2X2 U108 ( .A(n46), .B(n32), .Z(ab_5__0_) );
  HS65_GS_NOR2X2 U109 ( .A(A[4]), .B(n38), .Z(ab_4__8_) );
  HS65_GS_NOR2X2 U110 ( .A(n39), .B(n33), .Z(ab_4__7_) );
  HS65_GS_NOR2X2 U111 ( .A(n40), .B(n33), .Z(ab_4__6_) );
  HS65_GS_NOR2X2 U112 ( .A(n41), .B(n33), .Z(ab_4__5_) );
  HS65_GS_NOR2X2 U113 ( .A(n42), .B(n33), .Z(ab_4__4_) );
  HS65_GS_NOR2X2 U114 ( .A(n43), .B(n33), .Z(ab_4__3_) );
  HS65_GS_NOR2X2 U115 ( .A(n44), .B(n33), .Z(ab_4__2_) );
  HS65_GS_NOR2X2 U116 ( .A(n45), .B(n33), .Z(ab_4__1_) );
  HS65_GS_NOR2X2 U117 ( .A(n46), .B(n33), .Z(ab_4__0_) );
  HS65_GS_NOR2X2 U118 ( .A(A[3]), .B(n38), .Z(ab_3__8_) );
  HS65_GS_NOR2X2 U119 ( .A(n39), .B(n34), .Z(ab_3__7_) );
  HS65_GS_NOR2X2 U120 ( .A(n40), .B(n34), .Z(ab_3__6_) );
  HS65_GS_NOR2X2 U121 ( .A(n41), .B(n34), .Z(ab_3__5_) );
  HS65_GS_NOR2X2 U122 ( .A(n42), .B(n34), .Z(ab_3__4_) );
  HS65_GS_NOR2X2 U123 ( .A(n43), .B(n34), .Z(ab_3__3_) );
  HS65_GS_NOR2X2 U124 ( .A(n44), .B(n34), .Z(ab_3__2_) );
  HS65_GS_NOR2X2 U125 ( .A(n45), .B(n34), .Z(ab_3__1_) );
  HS65_GS_NOR2X2 U126 ( .A(n46), .B(n34), .Z(ab_3__0_) );
  HS65_GS_NOR2X2 U127 ( .A(A[2]), .B(n38), .Z(ab_2__8_) );
  HS65_GS_NOR2X2 U128 ( .A(n39), .B(n35), .Z(ab_2__7_) );
  HS65_GS_NOR2X2 U129 ( .A(n40), .B(n35), .Z(ab_2__6_) );
  HS65_GS_NOR2X2 U130 ( .A(n41), .B(n35), .Z(ab_2__5_) );
  HS65_GS_NOR2X2 U131 ( .A(n42), .B(n35), .Z(ab_2__4_) );
  HS65_GS_NOR2X2 U132 ( .A(n43), .B(n35), .Z(ab_2__3_) );
  HS65_GS_NOR2X2 U133 ( .A(n44), .B(n35), .Z(ab_2__2_) );
  HS65_GS_NOR2X2 U134 ( .A(n45), .B(n35), .Z(ab_2__1_) );
  HS65_GS_NOR2X2 U135 ( .A(n46), .B(n35), .Z(ab_2__0_) );
  HS65_GS_NOR2X2 U136 ( .A(A[1]), .B(n38), .Z(ab_1__8_) );
  HS65_GS_NOR2X2 U137 ( .A(n39), .B(n36), .Z(ab_1__7_) );
  HS65_GS_NOR2X2 U138 ( .A(n40), .B(n36), .Z(ab_1__6_) );
  HS65_GS_NOR2X2 U139 ( .A(n41), .B(n36), .Z(ab_1__5_) );
  HS65_GS_NOR2X2 U140 ( .A(n42), .B(n36), .Z(ab_1__4_) );
  HS65_GS_NOR2X2 U141 ( .A(n43), .B(n36), .Z(ab_1__3_) );
  HS65_GS_NOR2X2 U142 ( .A(n44), .B(n36), .Z(ab_1__2_) );
  HS65_GS_NOR2X2 U143 ( .A(n45), .B(n36), .Z(ab_1__1_) );
  HS65_GS_NOR2X2 U144 ( .A(n46), .B(n36), .Z(ab_1__0_) );
  HS65_GS_NOR2X2 U145 ( .A(n38), .B(n22), .Z(ab_16__8_) );
  HS65_GS_NOR2X2 U146 ( .A(B[7]), .B(n22), .Z(ab_16__7_) );
  HS65_GS_NOR2X2 U147 ( .A(B[6]), .B(n22), .Z(ab_16__6_) );
  HS65_GS_NOR2X2 U148 ( .A(B[5]), .B(n22), .Z(ab_16__5_) );
  HS65_GS_NOR2X2 U149 ( .A(B[4]), .B(n22), .Z(ab_16__4_) );
  HS65_GS_NOR2X2 U150 ( .A(B[3]), .B(n22), .Z(ab_16__3_) );
  HS65_GS_NOR2X2 U151 ( .A(B[2]), .B(n22), .Z(ab_16__2_) );
  HS65_GS_NOR2X2 U152 ( .A(B[1]), .B(n22), .Z(ab_16__1_) );
  HS65_GS_NOR2X2 U153 ( .A(B[0]), .B(n22), .Z(ab_16__0_) );
  HS65_GS_NOR2X2 U154 ( .A(A[15]), .B(n38), .Z(ab_15__8_) );
  HS65_GS_NOR2X2 U155 ( .A(n39), .B(n21), .Z(ab_15__7_) );
  HS65_GS_NOR2X2 U156 ( .A(n40), .B(n21), .Z(ab_15__6_) );
  HS65_GS_NOR2X2 U157 ( .A(n41), .B(n21), .Z(ab_15__5_) );
  HS65_GS_NOR2X2 U158 ( .A(n42), .B(n21), .Z(ab_15__4_) );
  HS65_GS_NOR2X2 U159 ( .A(n43), .B(n21), .Z(ab_15__3_) );
  HS65_GS_NOR2X2 U160 ( .A(n44), .B(n21), .Z(ab_15__2_) );
  HS65_GS_NOR2X2 U161 ( .A(n45), .B(n21), .Z(ab_15__1_) );
  HS65_GS_NOR2X2 U162 ( .A(n46), .B(n21), .Z(ab_15__0_) );
  HS65_GS_NOR2X2 U163 ( .A(A[14]), .B(n38), .Z(ab_14__8_) );
  HS65_GS_NOR2X2 U164 ( .A(n39), .B(n23), .Z(ab_14__7_) );
  HS65_GS_NOR2X2 U165 ( .A(n40), .B(n23), .Z(ab_14__6_) );
  HS65_GS_NOR2X2 U166 ( .A(n41), .B(n23), .Z(ab_14__5_) );
  HS65_GS_NOR2X2 U167 ( .A(n42), .B(n23), .Z(ab_14__4_) );
  HS65_GS_NOR2X2 U168 ( .A(n43), .B(n23), .Z(ab_14__3_) );
  HS65_GS_NOR2X2 U169 ( .A(n44), .B(n23), .Z(ab_14__2_) );
  HS65_GS_NOR2X2 U170 ( .A(n45), .B(n23), .Z(ab_14__1_) );
  HS65_GS_NOR2X2 U171 ( .A(n46), .B(n23), .Z(ab_14__0_) );
  HS65_GS_NOR2X2 U172 ( .A(A[13]), .B(n38), .Z(ab_13__8_) );
  HS65_GS_NOR2X2 U173 ( .A(n39), .B(n24), .Z(ab_13__7_) );
  HS65_GS_NOR2X2 U174 ( .A(n40), .B(n24), .Z(ab_13__6_) );
  HS65_GS_NOR2X2 U175 ( .A(n41), .B(n24), .Z(ab_13__5_) );
  HS65_GS_NOR2X2 U176 ( .A(n42), .B(n24), .Z(ab_13__4_) );
  HS65_GS_NOR2X2 U177 ( .A(n43), .B(n24), .Z(ab_13__3_) );
  HS65_GS_NOR2X2 U178 ( .A(n44), .B(n24), .Z(ab_13__2_) );
  HS65_GS_NOR2X2 U179 ( .A(n45), .B(n24), .Z(ab_13__1_) );
  HS65_GS_NOR2X2 U180 ( .A(n46), .B(n24), .Z(ab_13__0_) );
  HS65_GS_NOR2X2 U181 ( .A(A[12]), .B(n38), .Z(ab_12__8_) );
  HS65_GS_NOR2X2 U182 ( .A(n39), .B(n25), .Z(ab_12__7_) );
  HS65_GS_NOR2X2 U183 ( .A(n40), .B(n25), .Z(ab_12__6_) );
  HS65_GS_NOR2X2 U184 ( .A(n41), .B(n25), .Z(ab_12__5_) );
  HS65_GS_NOR2X2 U185 ( .A(n42), .B(n25), .Z(ab_12__4_) );
  HS65_GS_NOR2X2 U186 ( .A(n43), .B(n25), .Z(ab_12__3_) );
  HS65_GS_NOR2X2 U187 ( .A(n44), .B(n25), .Z(ab_12__2_) );
  HS65_GS_NOR2X2 U188 ( .A(n45), .B(n25), .Z(ab_12__1_) );
  HS65_GS_NOR2X2 U189 ( .A(n46), .B(n25), .Z(ab_12__0_) );
  HS65_GS_NOR2X2 U190 ( .A(A[11]), .B(n38), .Z(ab_11__8_) );
  HS65_GS_NOR2X2 U191 ( .A(n39), .B(n26), .Z(ab_11__7_) );
  HS65_GS_NOR2X2 U192 ( .A(n40), .B(n26), .Z(ab_11__6_) );
  HS65_GS_NOR2X2 U193 ( .A(n41), .B(n26), .Z(ab_11__5_) );
  HS65_GS_NOR2X2 U194 ( .A(n42), .B(n26), .Z(ab_11__4_) );
  HS65_GS_NOR2X2 U195 ( .A(n43), .B(n26), .Z(ab_11__3_) );
  HS65_GS_NOR2X2 U196 ( .A(n44), .B(n26), .Z(ab_11__2_) );
  HS65_GS_NOR2X2 U197 ( .A(n45), .B(n26), .Z(ab_11__1_) );
  HS65_GS_NOR2X2 U198 ( .A(n46), .B(n26), .Z(ab_11__0_) );
  HS65_GS_NOR2X2 U199 ( .A(A[10]), .B(n38), .Z(ab_10__8_) );
  HS65_GS_NOR2X2 U200 ( .A(n39), .B(n27), .Z(ab_10__7_) );
  HS65_GS_NOR2X2 U201 ( .A(n40), .B(n27), .Z(ab_10__6_) );
  HS65_GS_NOR2X2 U202 ( .A(n41), .B(n27), .Z(ab_10__5_) );
  HS65_GS_NOR2X2 U203 ( .A(n42), .B(n27), .Z(ab_10__4_) );
  HS65_GS_NOR2X2 U204 ( .A(n43), .B(n27), .Z(ab_10__3_) );
  HS65_GS_NOR2X2 U205 ( .A(n44), .B(n27), .Z(ab_10__2_) );
  HS65_GS_NOR2X2 U206 ( .A(n45), .B(n27), .Z(ab_10__1_) );
  HS65_GS_NOR2X2 U207 ( .A(n46), .B(n27), .Z(ab_10__0_) );
  HS65_GS_NOR2X2 U208 ( .A(A[0]), .B(n38), .Z(ab_0__8_) );
  HS65_GS_NOR2X2 U209 ( .A(n39), .B(n37), .Z(ab_0__7_) );
  HS65_GS_NOR2X2 U210 ( .A(n40), .B(n37), .Z(ab_0__6_) );
  HS65_GS_NOR2X2 U211 ( .A(n41), .B(n37), .Z(ab_0__5_) );
  HS65_GS_NOR2X2 U212 ( .A(n42), .B(n37), .Z(ab_0__4_) );
  HS65_GS_NOR2X2 U213 ( .A(n43), .B(n37), .Z(ab_0__3_) );
  HS65_GS_NOR2X2 U214 ( .A(n44), .B(n37), .Z(ab_0__2_) );
  HS65_GS_NOR2X2 U215 ( .A(n45), .B(n37), .Z(ab_0__1_) );
  HS65_GS_NOR2X2 U216 ( .A(n46), .B(n37), .Z(PRODUCT[0]) );
  HS65_GS_IVX2 U218 ( .A(CARRYB_16__8_), .Z(A1_23_) );
endmodule


module omsp_multiplier ( per_dout, mclk, per_addr, per_din, per_en, per_we, 
        puc_rst, scan_enable );
  output [15:0] per_dout;
  input [13:0] per_addr;
  input [15:0] per_din;
  input [1:0] per_we;
  input mclk, per_en, puc_rst, scan_enable;
  wire   reshi_0_, sign_sel, op1_xp_16_, n17, n31, n32, n33, n34, n35, n36,
         n38, n40, n42, n43, n44, n46, n47, n48, n50, n51, n52, n54, n55, n56,
         n58, n59, n60, n62, n63, n64, n66, n67, n68, n70, n71, n72, n74, n75,
         n76, n78, n79, n80, n82, n83, n84, n86, n87, n88, n90, n91, n92, n94,
         n95, n96, n98, n99, n103, n104, n105, n107, n109, n110, n111, n115,
         n116, n117, n118, n137, n146, n148, n150, n151, n153, n168, n179,
         n180, n181, n182, n183, n185, n193, n202, n203, n205, n207, n214,
         n215, n216, n217, n218, n219, n220, n221, n222, n223, n224, n225,
         n226, n227, n228, n229, n230, n231, n232, n233, n234, n235, n236,
         n237, n238, n239, n240, n242, n244, n245, n246, n247, n248, n249,
         n250, n251, n252, n253, n254, n255, n256, n257, n258, n259, n260,
         n261, n262, n263, n264, n265, n266, n267, n268, n269, n270, n271,
         n272, n273, n274, n275, n276, n277, n278, n279, n280, n281, n282,
         n283, n284, n285, n286, n287, n288, n289, n290, n291, n292, n293,
         n294, n295, n296, n297, n298, n299, n300, n301, n302, n303, n2, n3,
         n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n18, n19,
         n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n37, n39, n41,
         n45, n49, n53, n57, n61, n65, n69, n73, n77, n81, n85, n89, n93, n97,
         n100, n101, n102, n106, n108, n112, n113, n114, n119, n120, n121,
         n122, n123, n124, n125, n126, n127, n128, n129, n130, n131, n132,
         n133, n134, n135, n136, n138, n139, n140, n141, n142, n143, n144,
         n145, n147, n149, n152, n154, n155, n156, n157, n158, n159, n160,
         n161, n162, n163, n164, n165, n166, n167, n169, n170, n171, n172,
         n173, n174, n175, n176, n177, n178, n184, n186, n187, n188, n189,
         n190, n191, n192, n194, n195, n196, n197, n198, n199, n200, n201,
         n204, n206, n208, n209, n210, n211, n212, n213, n241, n243, n304,
         n305, n306, n307, n309, n310, n311, n312, n313, n314, n315, n316,
         n317, n318, n319, n320, n321, n322, n323, n324,
         SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2;
  wire   [15:0] op1;
  wire   [15:0] op2;
  wire   [15:1] reslo;
  wire   [1:0] sumext_s;
  wire   [1:0] cycle;
  wire   [8:0] op2_xp;
  wire   [23:0] product;
  wire   [31:0] product_xp;
  wire   [32:0] result_nxt;

  omsp_multiplier_DW01_add_0 add_402 ( .A({1'b0, n324, n323, n322, n321, n320, 
        n319, n318, n317, n316, n315, n314, n313, n312, n311, n310, reshi_0_, 
        reslo, n309}), .B({1'b0, product_xp}), .CI(1'b0), .SUM(result_nxt) );
  omsp_multiplier_DW02_mult_0 mult_396 ( .A({op1_xp_16_, op1}), .B(op2_xp), 
        .TC(1'b1), .PRODUCT({SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2, 
        product}) );
  HS65_GS_DFPRQNX4 acc_sel_reg ( .D(n278), .CP(mclk), .RN(n3), .QN(n242) );
  HS65_GS_DFPRQNX4 reshi_reg_7_ ( .D(n254), .CP(mclk), .RN(n4), .QN(n225) );
  HS65_GS_DFPRQNX4 reshi_reg_6_ ( .D(n255), .CP(mclk), .RN(n5), .QN(n226) );
  HS65_GS_DFPRQNX4 reshi_reg_5_ ( .D(n256), .CP(mclk), .RN(n3), .QN(n227) );
  HS65_GS_DFPRQNX4 reshi_reg_4_ ( .D(n257), .CP(mclk), .RN(n4), .QN(n228) );
  HS65_GS_DFPRQNX4 reshi_reg_3_ ( .D(n258), .CP(mclk), .RN(n5), .QN(n229) );
  HS65_GS_DFPRQNX4 reshi_reg_2_ ( .D(n259), .CP(mclk), .RN(n2), .QN(n230) );
  HS65_GS_DFPRQNX4 reshi_reg_1_ ( .D(n260), .CP(mclk), .RN(n3), .QN(n231) );
  HS65_GS_DFPRQNX4 reslo_reg_0_ ( .D(n277), .CP(mclk), .RN(n4), .QN(n232) );
  HS65_GS_DFPRQNX4 reshi_reg_15_ ( .D(n246), .CP(mclk), .RN(n4), .QN(n217) );
  HS65_GS_DFPRQNX4 reshi_reg_14_ ( .D(n247), .CP(mclk), .RN(n3), .QN(n218) );
  HS65_GS_DFPRQNX4 reshi_reg_13_ ( .D(n248), .CP(mclk), .RN(n2), .QN(n219) );
  HS65_GS_DFPRQNX4 reshi_reg_12_ ( .D(n249), .CP(mclk), .RN(n5), .QN(n220) );
  HS65_GS_DFPRQNX4 reshi_reg_11_ ( .D(n250), .CP(mclk), .RN(n4), .QN(n221) );
  HS65_GS_DFPRQNX4 reshi_reg_10_ ( .D(n251), .CP(mclk), .RN(n5), .QN(n222) );
  HS65_GS_DFPRQNX4 reshi_reg_9_ ( .D(n252), .CP(mclk), .RN(n2), .QN(n223) );
  HS65_GS_DFPRQNX4 reshi_reg_8_ ( .D(n253), .CP(mclk), .RN(n3), .QN(n224) );
  HS65_GS_DFPRQX4 cycle_reg_0_ ( .D(n6), .CP(mclk), .RN(n2), .Q(cycle[0]) );
  HS65_GS_DFPRQX4 sign_sel_reg ( .D(n279), .CP(mclk), .RN(n5), .Q(sign_sel) );
  HS65_GS_DFPRQX4 op2_reg_7_ ( .D(n240), .CP(mclk), .RN(n4), .Q(op2[7]) );
  HS65_GS_DFPRQX4 op2_reg_6_ ( .D(n239), .CP(mclk), .RN(n3), .Q(op2[6]) );
  HS65_GS_DFPRQX4 op2_reg_5_ ( .D(n238), .CP(mclk), .RN(n2), .Q(op2[5]) );
  HS65_GS_DFPRQX4 op2_reg_4_ ( .D(n237), .CP(mclk), .RN(n5), .Q(op2[4]) );
  HS65_GS_DFPRQX4 op2_reg_3_ ( .D(n236), .CP(mclk), .RN(n4), .Q(op2[3]) );
  HS65_GS_DFPRQX4 op2_reg_2_ ( .D(n235), .CP(mclk), .RN(n3), .Q(op2[2]) );
  HS65_GS_DFPRQX4 op2_reg_1_ ( .D(n234), .CP(mclk), .RN(n2), .Q(op2[1]) );
  HS65_GS_DFPRQX4 op2_reg_0_ ( .D(n233), .CP(mclk), .RN(n5), .Q(op2[0]) );
  HS65_GS_DFPRQX4 op1_reg_15_ ( .D(n303), .CP(mclk), .RN(n4), .Q(op1[15]) );
  HS65_GS_DFPRQX4 op1_reg_14_ ( .D(n302), .CP(mclk), .RN(n3), .Q(op1[14]) );
  HS65_GS_DFPRQX4 op1_reg_13_ ( .D(n301), .CP(mclk), .RN(n2), .Q(op1[13]) );
  HS65_GS_DFPRQX4 op1_reg_12_ ( .D(n300), .CP(mclk), .RN(n5), .Q(op1[12]) );
  HS65_GS_DFPRQX4 op1_reg_11_ ( .D(n299), .CP(mclk), .RN(n4), .Q(op1[11]) );
  HS65_GS_DFPRQX4 op1_reg_10_ ( .D(n298), .CP(mclk), .RN(n3), .Q(op1[10]) );
  HS65_GS_DFPRQX4 op1_reg_9_ ( .D(n297), .CP(mclk), .RN(n2), .Q(op1[9]) );
  HS65_GS_DFPRQX4 op1_reg_8_ ( .D(n296), .CP(mclk), .RN(n5), .Q(op1[8]) );
  HS65_GS_DFPRQX4 op1_reg_7_ ( .D(n295), .CP(mclk), .RN(n4), .Q(op1[7]) );
  HS65_GS_DFPRQX4 op1_reg_6_ ( .D(n294), .CP(mclk), .RN(n3), .Q(op1[6]) );
  HS65_GS_DFPRQX4 op1_reg_5_ ( .D(n293), .CP(mclk), .RN(n2), .Q(op1[5]) );
  HS65_GS_DFPRQX4 op1_reg_4_ ( .D(n292), .CP(mclk), .RN(n5), .Q(op1[4]) );
  HS65_GS_DFPRQX4 op1_reg_3_ ( .D(n291), .CP(mclk), .RN(n4), .Q(op1[3]) );
  HS65_GS_DFPRQX4 op1_reg_2_ ( .D(n290), .CP(mclk), .RN(n3), .Q(op1[2]) );
  HS65_GS_DFPRQX4 op1_reg_1_ ( .D(n289), .CP(mclk), .RN(n2), .Q(op1[1]) );
  HS65_GS_DFPRQX4 op1_reg_0_ ( .D(n288), .CP(mclk), .RN(n5), .Q(op1[0]) );
  HS65_GS_DFPRQX4 sumext_s_reg_0_ ( .D(n245), .CP(mclk), .RN(n4), .Q(
        sumext_s[0]) );
  HS65_GS_DFPRQX4 sumext_s_reg_1_ ( .D(n244), .CP(mclk), .RN(n3), .Q(
        sumext_s[1]) );
  HS65_GS_DFPRQX4 op2_reg_15_ ( .D(n287), .CP(mclk), .RN(n2), .Q(op2[15]) );
  HS65_GS_DFPRQX4 op2_reg_14_ ( .D(n286), .CP(mclk), .RN(n5), .Q(op2[14]) );
  HS65_GS_DFPRQX4 op2_reg_13_ ( .D(n285), .CP(mclk), .RN(n4), .Q(op2[13]) );
  HS65_GS_DFPRQX4 op2_reg_12_ ( .D(n284), .CP(mclk), .RN(n3), .Q(op2[12]) );
  HS65_GS_DFPRQX4 op2_reg_11_ ( .D(n283), .CP(mclk), .RN(n2), .Q(op2[11]) );
  HS65_GS_DFPRQX4 op2_reg_10_ ( .D(n282), .CP(mclk), .RN(n5), .Q(op2[10]) );
  HS65_GS_DFPRQX4 op2_reg_9_ ( .D(n281), .CP(mclk), .RN(n4), .Q(op2[9]) );
  HS65_GS_DFPRQX4 op2_reg_8_ ( .D(n280), .CP(mclk), .RN(n3), .Q(op2[8]) );
  HS65_GS_DFPRQX4 reshi_reg_0_ ( .D(n261), .CP(mclk), .RN(n2), .Q(reshi_0_) );
  HS65_GS_DFPRQX4 reslo_reg_7_ ( .D(n270), .CP(mclk), .RN(n5), .Q(reslo[7]) );
  HS65_GS_DFPRQX4 reslo_reg_6_ ( .D(n271), .CP(mclk), .RN(n4), .Q(reslo[6]) );
  HS65_GS_DFPRQX4 reslo_reg_5_ ( .D(n272), .CP(mclk), .RN(n3), .Q(reslo[5]) );
  HS65_GS_DFPRQX4 reslo_reg_4_ ( .D(n273), .CP(mclk), .RN(n2), .Q(reslo[4]) );
  HS65_GS_DFPRQX4 reslo_reg_3_ ( .D(n274), .CP(mclk), .RN(n5), .Q(reslo[3]) );
  HS65_GS_DFPRQX4 reslo_reg_2_ ( .D(n275), .CP(mclk), .RN(n4), .Q(reslo[2]) );
  HS65_GS_DFPRQX4 reslo_reg_1_ ( .D(n276), .CP(mclk), .RN(n3), .Q(reslo[1]) );
  HS65_GS_DFPRQX4 reslo_reg_15_ ( .D(n262), .CP(mclk), .RN(n2), .Q(reslo[15])
         );
  HS65_GS_DFPRQX4 reslo_reg_14_ ( .D(n263), .CP(mclk), .RN(n5), .Q(reslo[14])
         );
  HS65_GS_DFPRQX4 reslo_reg_13_ ( .D(n264), .CP(mclk), .RN(n4), .Q(reslo[13])
         );
  HS65_GS_DFPRQX4 reslo_reg_12_ ( .D(n265), .CP(mclk), .RN(n3), .Q(reslo[12])
         );
  HS65_GS_DFPRQX4 reslo_reg_11_ ( .D(n266), .CP(mclk), .RN(n2), .Q(reslo[11])
         );
  HS65_GS_DFPRQX4 reslo_reg_10_ ( .D(n267), .CP(mclk), .RN(n5), .Q(reslo[10])
         );
  HS65_GS_DFPRQX4 reslo_reg_9_ ( .D(n268), .CP(mclk), .RN(n4), .Q(reslo[9]) );
  HS65_GS_DFPRQX4 reslo_reg_8_ ( .D(n269), .CP(mclk), .RN(n3), .Q(reslo[8]) );
  HS65_GS_DFPRQX4 cycle_reg_1_ ( .D(cycle[0]), .CP(mclk), .RN(n2), .Q(cycle[1]) );
  HS65_GS_MUX21X4 U3 ( .D0(n306), .D1(n111), .S0(cycle[1]), .Z(n107) );
  HS65_GS_NOR3AX2 U4 ( .A(n115), .B(n196), .C(n133), .Z(n8) );
  HS65_GS_NOR4ABX4 U5 ( .A(n133), .B(n136), .C(n138), .D(n110), .Z(n7) );
  HS65_GS_IVX2 U6 ( .A(puc_rst), .Z(n2) );
  HS65_GS_IVX2 U7 ( .A(puc_rst), .Z(n3) );
  HS65_GS_IVX2 U8 ( .A(puc_rst), .Z(n4) );
  HS65_GS_IVX2 U9 ( .A(puc_rst), .Z(n5) );
  HS65_GS_IVX9 U11 ( .A(n137), .Z(n6) );
  HS65_GS_NAND3X5 U12 ( .A(per_addr[0]), .B(per_addr[2]), .C(n109), .Z(n36) );
  HS65_GS_NAND3X5 U13 ( .A(per_addr[0]), .B(n98), .C(n99), .Z(n38) );
  HS65_GS_IVX9 U14 ( .A(product[20]), .Z(n29) );
  HS65_GS_IVX9 U15 ( .A(product[21]), .Z(n27) );
  HS65_GS_IVX9 U16 ( .A(product[19]), .Z(n37) );
  HS65_GS_IVX9 U17 ( .A(product[18]), .Z(n41) );
  HS65_GS_IVX9 U18 ( .A(product[17]), .Z(n49) );
  HS65_GS_IVX9 U19 ( .A(n137), .Z(n132) );
  HS65_GS_IVX9 U20 ( .A(product[22]), .Z(n25) );
  HS65_GS_IVX9 U21 ( .A(product[16]), .Z(n57) );
  HS65_GS_IVX9 U22 ( .A(n168), .Z(n134) );
  HS65_GS_IVX9 U23 ( .A(n98), .Z(n135) );
  HS65_GS_NAND2X7 U24 ( .A(n202), .B(n133), .Z(n137) );
  HS65_GS_IVX9 U25 ( .A(n203), .Z(n139) );
  HS65_GS_IVX9 U26 ( .A(product[10]), .Z(n101) );
  HS65_GS_IVX9 U27 ( .A(product[1]), .Z(n130) );
  HS65_GS_IVX9 U28 ( .A(product[23]), .Z(n23) );
  HS65_GS_IVX9 U29 ( .A(product[9]), .Z(n106) );
  HS65_GS_IVX9 U30 ( .A(n9), .Z(n10) );
  HS65_GS_IVX9 U31 ( .A(n9), .Z(n11) );
  HS65_GS_IVX9 U32 ( .A(product[12]), .Z(n89) );
  HS65_GS_IVX9 U33 ( .A(product[14]), .Z(n73) );
  HS65_GS_IVX9 U34 ( .A(product[15]), .Z(n65) );
  HS65_GS_IVX9 U35 ( .A(product[11]), .Z(n97) );
  HS65_GS_IVX9 U36 ( .A(product[13]), .Z(n81) );
  HS65_GS_IVX9 U37 ( .A(n148), .Z(n12) );
  HS65_GS_IVX9 U38 ( .A(product[2]), .Z(n128) );
  HS65_GS_IVX9 U39 ( .A(product[3]), .Z(n126) );
  HS65_GS_IVX9 U40 ( .A(product[4]), .Z(n124) );
  HS65_GS_IVX9 U41 ( .A(product[5]), .Z(n122) );
  HS65_GS_IVX9 U42 ( .A(product[6]), .Z(n120) );
  HS65_GS_IVX9 U43 ( .A(product[7]), .Z(n114) );
  HS65_GS_IVX9 U44 ( .A(product[8]), .Z(n112) );
  HS65_GS_NOR2X6 U45 ( .A(cycle[0]), .B(n171), .Z(product_xp[0]) );
  HS65_GS_IVX9 U46 ( .A(per_din[8]), .Z(n159) );
  HS65_GS_IVX9 U47 ( .A(per_din[9]), .Z(n160) );
  HS65_GS_IVX9 U48 ( .A(per_din[10]), .Z(n141) );
  HS65_GS_IVX9 U49 ( .A(per_din[11]), .Z(n142) );
  HS65_GS_IVX9 U50 ( .A(per_din[12]), .Z(n143) );
  HS65_GS_IVX9 U51 ( .A(per_din[13]), .Z(n144) );
  HS65_GS_IVX9 U52 ( .A(per_din[14]), .Z(n145) );
  HS65_GS_IVX9 U53 ( .A(per_din[15]), .Z(n147) );
  HS65_GS_IVX9 U54 ( .A(per_din[0]), .Z(n140) );
  HS65_GS_IVX9 U55 ( .A(per_din[1]), .Z(n149) );
  HS65_GS_IVX9 U56 ( .A(per_din[2]), .Z(n152) );
  HS65_GS_IVX9 U57 ( .A(per_din[3]), .Z(n154) );
  HS65_GS_IVX9 U58 ( .A(per_din[4]), .Z(n155) );
  HS65_GS_IVX9 U59 ( .A(per_din[5]), .Z(n156) );
  HS65_GS_IVX9 U60 ( .A(per_din[6]), .Z(n157) );
  HS65_GS_IVX9 U61 ( .A(per_din[7]), .Z(n158) );
  HS65_GS_NOR3X4 U62 ( .A(n196), .B(per_addr[0]), .C(n135), .Z(n33) );
  HS65_GS_NAND3X5 U63 ( .A(n193), .B(n179), .C(n185), .Z(n182) );
  HS65_GS_NAND3X5 U64 ( .A(n179), .B(n134), .C(n153), .Z(n150) );
  HS65_GS_NOR3X4 U65 ( .A(n138), .B(per_addr[1]), .C(n181), .Z(n202) );
  HS65_GS_NOR2X6 U66 ( .A(n110), .B(per_addr[2]), .Z(n40) );
  HS65_GS_NOR3X4 U67 ( .A(n136), .B(n110), .C(n138), .Z(n98) );
  HS65_GS_NOR2X6 U68 ( .A(n181), .B(per_addr[2]), .Z(n203) );
  HS65_GS_OAI222X2 U69 ( .A(n243), .B(n182), .C(n147), .D(n183), .E(n61), .F(
        n185), .Z(n262) );
  HS65_GS_IVX9 U70 ( .A(result_nxt[15]), .Z(n61) );
  HS65_GS_OAI222X2 U71 ( .A(n241), .B(n182), .C(n145), .D(n183), .E(n69), .F(
        n185), .Z(n263) );
  HS65_GS_IVX9 U72 ( .A(result_nxt[14]), .Z(n69) );
  HS65_GS_OAI222X2 U73 ( .A(n213), .B(n182), .C(n144), .D(n183), .E(n77), .F(
        n185), .Z(n264) );
  HS65_GS_IVX9 U74 ( .A(result_nxt[13]), .Z(n77) );
  HS65_GS_OAI222X2 U75 ( .A(n212), .B(n182), .C(n143), .D(n183), .E(n85), .F(
        n185), .Z(n265) );
  HS65_GS_IVX9 U76 ( .A(result_nxt[12]), .Z(n85) );
  HS65_GS_OAI222X2 U77 ( .A(n211), .B(n182), .C(n142), .D(n183), .E(n93), .F(
        n185), .Z(n266) );
  HS65_GS_IVX9 U78 ( .A(result_nxt[11]), .Z(n93) );
  HS65_GS_OAI222X2 U79 ( .A(n210), .B(n182), .C(n141), .D(n183), .E(n100), .F(
        n185), .Z(n267) );
  HS65_GS_IVX9 U80 ( .A(result_nxt[10]), .Z(n100) );
  HS65_GS_OAI222X2 U81 ( .A(n209), .B(n182), .C(n160), .D(n183), .E(n102), .F(
        n185), .Z(n268) );
  HS65_GS_IVX9 U82 ( .A(result_nxt[9]), .Z(n102) );
  HS65_GS_OAI222X2 U83 ( .A(n208), .B(n182), .C(n159), .D(n183), .E(n108), .F(
        n185), .Z(n269) );
  HS65_GS_IVX9 U84 ( .A(result_nxt[8]), .Z(n108) );
  HS65_GS_NAND2X7 U85 ( .A(per_we[1]), .B(n168), .Z(n151) );
  HS65_GS_OAI222X2 U86 ( .A(n113), .B(n185), .C(n206), .D(n182), .E(n158), .F(
        n193), .Z(n270) );
  HS65_GS_IVX9 U87 ( .A(result_nxt[7]), .Z(n113) );
  HS65_GS_OAI222X2 U88 ( .A(n119), .B(n185), .C(n204), .D(n182), .E(n157), .F(
        n193), .Z(n271) );
  HS65_GS_IVX9 U89 ( .A(result_nxt[6]), .Z(n119) );
  HS65_GS_OAI222X2 U90 ( .A(n121), .B(n185), .C(n201), .D(n182), .E(n156), .F(
        n193), .Z(n272) );
  HS65_GS_IVX9 U91 ( .A(result_nxt[5]), .Z(n121) );
  HS65_GS_OAI222X2 U92 ( .A(n123), .B(n185), .C(n200), .D(n182), .E(n155), .F(
        n193), .Z(n273) );
  HS65_GS_IVX9 U93 ( .A(result_nxt[4]), .Z(n123) );
  HS65_GS_OAI222X2 U94 ( .A(n125), .B(n185), .C(n199), .D(n182), .E(n154), .F(
        n193), .Z(n274) );
  HS65_GS_IVX9 U95 ( .A(result_nxt[3]), .Z(n125) );
  HS65_GS_OAI222X2 U96 ( .A(n127), .B(n185), .C(n198), .D(n182), .E(n152), .F(
        n193), .Z(n275) );
  HS65_GS_IVX9 U97 ( .A(result_nxt[2]), .Z(n127) );
  HS65_GS_OAI222X2 U98 ( .A(n129), .B(n185), .C(n197), .D(n182), .E(n149), .F(
        n193), .Z(n276) );
  HS65_GS_IVX9 U99 ( .A(result_nxt[1]), .Z(n129) );
  HS65_GS_NAND2X7 U100 ( .A(n203), .B(per_we[1]), .Z(n207) );
  HS65_GS_NAND2X7 U101 ( .A(per_we[1]), .B(n132), .Z(n205) );
  HS65_GS_IVX9 U102 ( .A(per_addr[0]), .Z(n133) );
  HS65_GS_OAI32X5 U103 ( .A(n132), .B(n146), .C(n111), .D(n306), .E(n131), .Z(
        n245) );
  HS65_GS_OAI32X5 U104 ( .A(n12), .B(n146), .C(n132), .D(n131), .E(n305), .Z(
        n244) );
  HS65_GS_NOR4ABX2 U105 ( .A(per_addr[2]), .B(per_addr[1]), .C(per_addr[0]), 
        .D(n181), .Z(n168) );
  HS65_GS_NAND2X7 U106 ( .A(n180), .B(n193), .Z(n185) );
  HS65_GS_NAND2X7 U107 ( .A(n180), .B(n134), .Z(n153) );
  HS65_GS_NAND2X7 U108 ( .A(n202), .B(per_addr[0]), .Z(n193) );
  HS65_GS_IVX9 U109 ( .A(per_addr[2]), .Z(n138) );
  HS65_GS_NAND2AX7 U110 ( .A(n193), .B(per_we[1]), .Z(n183) );
  HS65_GS_OAI22X6 U111 ( .A(n203), .B(n195), .C(n133), .D(n139), .Z(n279) );
  HS65_GS_OAI22X6 U112 ( .A(n203), .B(n162), .C(n147), .D(n207), .Z(n303) );
  HS65_GS_OAI22X6 U113 ( .A(n132), .B(n194), .C(n137), .D(n140), .Z(n233) );
  HS65_GS_OAI22X6 U114 ( .A(n132), .B(n192), .C(n137), .D(n149), .Z(n234) );
  HS65_GS_OAI22X6 U115 ( .A(n132), .B(n191), .C(n137), .D(n152), .Z(n235) );
  HS65_GS_OAI22X6 U116 ( .A(n132), .B(n190), .C(n137), .D(n154), .Z(n236) );
  HS65_GS_OAI22X6 U117 ( .A(n132), .B(n189), .C(n137), .D(n155), .Z(n237) );
  HS65_GS_OAI22X6 U118 ( .A(n132), .B(n188), .C(n137), .D(n156), .Z(n238) );
  HS65_GS_OAI22X6 U119 ( .A(n132), .B(n187), .C(n137), .D(n157), .Z(n239) );
  HS65_GS_OAI22X6 U120 ( .A(n132), .B(n186), .C(n137), .D(n158), .Z(n240) );
  HS65_GS_OAI22X6 U121 ( .A(n132), .B(n184), .C(n159), .D(n205), .Z(n280) );
  HS65_GS_OAI22X6 U122 ( .A(n132), .B(n178), .C(n160), .D(n205), .Z(n281) );
  HS65_GS_OAI22X6 U123 ( .A(n132), .B(n177), .C(n141), .D(n205), .Z(n282) );
  HS65_GS_OAI22X6 U124 ( .A(n132), .B(n176), .C(n142), .D(n205), .Z(n283) );
  HS65_GS_OAI22X6 U125 ( .A(n132), .B(n175), .C(n143), .D(n205), .Z(n284) );
  HS65_GS_OAI22X6 U126 ( .A(n132), .B(n174), .C(n144), .D(n205), .Z(n285) );
  HS65_GS_OAI22X6 U127 ( .A(n132), .B(n173), .C(n145), .D(n205), .Z(n286) );
  HS65_GS_OAI22X6 U128 ( .A(n6), .B(n172), .C(n147), .D(n205), .Z(n287) );
  HS65_GS_OAI21X3 U129 ( .A(n36), .B(n197), .C(n38), .Z(n72) );
  HS65_GS_OAI21X3 U130 ( .A(n36), .B(n198), .C(n38), .Z(n68) );
  HS65_GS_OAI21X3 U131 ( .A(n36), .B(n199), .C(n38), .Z(n64) );
  HS65_GS_OAI21X3 U132 ( .A(n36), .B(n200), .C(n38), .Z(n60) );
  HS65_GS_OAI21X3 U133 ( .A(n36), .B(n201), .C(n38), .Z(n56) );
  HS65_GS_OAI21X3 U134 ( .A(n36), .B(n204), .C(n38), .Z(n52) );
  HS65_GS_OAI21X3 U135 ( .A(n36), .B(n206), .C(n38), .Z(n48) );
  HS65_GS_OAI21X3 U136 ( .A(n36), .B(n208), .C(n38), .Z(n44) );
  HS65_GS_OAI21X3 U137 ( .A(n36), .B(n209), .C(n38), .Z(n35) );
  HS65_GS_OAI21X3 U138 ( .A(n36), .B(n210), .C(n38), .Z(n96) );
  HS65_GS_OAI21X3 U139 ( .A(n36), .B(n211), .C(n38), .Z(n92) );
  HS65_GS_OAI21X3 U140 ( .A(n36), .B(n212), .C(n38), .Z(n88) );
  HS65_GS_OAI21X3 U141 ( .A(n36), .B(n213), .C(n38), .Z(n84) );
  HS65_GS_OAI21X3 U142 ( .A(n36), .B(n241), .C(n38), .Z(n80) );
  HS65_GS_OAI21X3 U143 ( .A(n36), .B(n243), .C(n38), .Z(n76) );
  HS65_GS_IVX9 U144 ( .A(n146), .Z(n131) );
  HS65_GS_NOR3X4 U145 ( .A(n138), .B(per_addr[1]), .C(n110), .Z(n115) );
  HS65_GS_NOR3X4 U146 ( .A(n172), .B(n10), .C(n195), .Z(op2_xp[8]) );
  HS65_GS_OAI22X6 U147 ( .A(n11), .B(n172), .C(n9), .D(n186), .Z(op2_xp[7]) );
  HS65_GS_OAI22X6 U148 ( .A(n10), .B(n178), .C(cycle[0]), .D(n192), .Z(
        op2_xp[1]) );
  HS65_GS_OAI22X6 U149 ( .A(n10), .B(n177), .C(cycle[0]), .D(n191), .Z(
        op2_xp[2]) );
  HS65_GS_OAI22X6 U150 ( .A(n10), .B(n176), .C(cycle[0]), .D(n190), .Z(
        op2_xp[3]) );
  HS65_GS_OAI22X6 U151 ( .A(n11), .B(n184), .C(cycle[0]), .D(n194), .Z(
        op2_xp[0]) );
  HS65_GS_OAI22X6 U152 ( .A(n11), .B(n175), .C(cycle[0]), .D(n189), .Z(
        op2_xp[4]) );
  HS65_GS_OAI22X6 U153 ( .A(n11), .B(n173), .C(n9), .D(n187), .Z(op2_xp[6]) );
  HS65_GS_OAI22X6 U154 ( .A(n11), .B(n174), .C(cycle[0]), .D(n188), .Z(
        op2_xp[5]) );
  HS65_GS_NOR2X6 U155 ( .A(n13), .B(n195), .Z(n148) );
  HS65_GS_IVX9 U156 ( .A(per_addr[1]), .Z(n136) );
  HS65_GS_IVX9 U157 ( .A(result_nxt[31]), .Z(n13) );
  HS65_GS_NOR2X6 U158 ( .A(n195), .B(n162), .Z(op1_xp_16_) );
  HS65_GS_IVX9 U159 ( .A(product[0]), .Z(n171) );
  HS65_GS_NOR3X4 U160 ( .A(cycle[1]), .B(per_addr[0]), .C(n135), .Z(n34) );
  HS65_GS_NOR3X4 U161 ( .A(n110), .B(per_addr[1]), .C(cycle[1]), .Z(n109) );
  HS65_GS_NOR3X4 U162 ( .A(n9), .B(cycle[1]), .C(n132), .Z(n146) );
  HS65_GS_OAI222X2 U163 ( .A(n217), .B(n150), .C(n151), .D(n147), .E(n13), .F(
        n153), .Z(n246) );
  HS65_GS_OAI222X2 U164 ( .A(n218), .B(n150), .C(n151), .D(n145), .E(n14), .F(
        n153), .Z(n247) );
  HS65_GS_IVX9 U165 ( .A(result_nxt[30]), .Z(n14) );
  HS65_GS_OAI222X2 U166 ( .A(n219), .B(n150), .C(n151), .D(n144), .E(n15), .F(
        n153), .Z(n248) );
  HS65_GS_IVX9 U167 ( .A(result_nxt[29]), .Z(n15) );
  HS65_GS_OAI222X2 U168 ( .A(n220), .B(n150), .C(n151), .D(n143), .E(n16), .F(
        n153), .Z(n249) );
  HS65_GS_IVX9 U169 ( .A(result_nxt[28]), .Z(n16) );
  HS65_GS_OAI222X2 U170 ( .A(n221), .B(n150), .C(n151), .D(n142), .E(n18), .F(
        n153), .Z(n250) );
  HS65_GS_IVX9 U171 ( .A(result_nxt[27]), .Z(n18) );
  HS65_GS_OAI222X2 U172 ( .A(n222), .B(n150), .C(n151), .D(n141), .E(n19), .F(
        n153), .Z(n251) );
  HS65_GS_IVX9 U173 ( .A(result_nxt[26]), .Z(n19) );
  HS65_GS_OAI222X2 U174 ( .A(n223), .B(n150), .C(n151), .D(n160), .E(n20), .F(
        n153), .Z(n252) );
  HS65_GS_IVX9 U175 ( .A(result_nxt[25]), .Z(n20) );
  HS65_GS_OAI222X2 U176 ( .A(n224), .B(n150), .C(n151), .D(n159), .E(n21), .F(
        n153), .Z(n253) );
  HS65_GS_IVX9 U177 ( .A(result_nxt[24]), .Z(n21) );
  HS65_GS_AOI22X6 U178 ( .A(cycle[1]), .B(n12), .C(n196), .D(n305), .Z(n99) );
  HS65_GS_NAND4ABX3 U179 ( .A(per_addr[8]), .B(per_addr[6]), .C(n116), .D(n117), .Z(n110) );
  HS65_GS_NOR3X4 U180 ( .A(per_addr[9]), .B(per_we[1]), .C(per_we[0]), .Z(n116) );
  HS65_GS_NOR4ABX2 U181 ( .A(n118), .B(n307), .C(per_addr[5]), .D(per_addr[13]), .Z(n117) );
  HS65_GS_IVX9 U182 ( .A(per_addr[12]), .Z(n307) );
  HS65_GS_OAI222X2 U183 ( .A(n161), .B(n185), .C(n232), .D(n182), .E(n140), 
        .F(n193), .Z(n277) );
  HS65_GS_IVX9 U184 ( .A(result_nxt[0]), .Z(n161) );
  HS65_GS_OAI222X2 U185 ( .A(n22), .B(n153), .C(n225), .D(n150), .E(n158), .F(
        n134), .Z(n254) );
  HS65_GS_IVX9 U186 ( .A(result_nxt[23]), .Z(n22) );
  HS65_GS_OAI222X2 U187 ( .A(n24), .B(n153), .C(n226), .D(n150), .E(n157), .F(
        n134), .Z(n255) );
  HS65_GS_IVX9 U188 ( .A(result_nxt[22]), .Z(n24) );
  HS65_GS_OAI222X2 U189 ( .A(n26), .B(n153), .C(n227), .D(n150), .E(n156), .F(
        n134), .Z(n256) );
  HS65_GS_IVX9 U190 ( .A(result_nxt[21]), .Z(n26) );
  HS65_GS_OAI222X2 U191 ( .A(n28), .B(n153), .C(n228), .D(n150), .E(n155), .F(
        n134), .Z(n257) );
  HS65_GS_IVX9 U192 ( .A(result_nxt[20]), .Z(n28) );
  HS65_GS_OAI222X2 U193 ( .A(n30), .B(n153), .C(n229), .D(n150), .E(n154), .F(
        n134), .Z(n258) );
  HS65_GS_IVX9 U194 ( .A(result_nxt[19]), .Z(n30) );
  HS65_GS_OAI222X2 U195 ( .A(n39), .B(n153), .C(n230), .D(n150), .E(n152), .F(
        n134), .Z(n259) );
  HS65_GS_IVX9 U196 ( .A(result_nxt[18]), .Z(n39) );
  HS65_GS_OAI222X2 U197 ( .A(n45), .B(n153), .C(n231), .D(n150), .E(n149), .F(
        n134), .Z(n260) );
  HS65_GS_IVX9 U198 ( .A(result_nxt[17]), .Z(n45) );
  HS65_GS_OAI222X2 U199 ( .A(n53), .B(n153), .C(n304), .D(n150), .E(n140), .F(
        n134), .Z(n261) );
  HS65_GS_IVX9 U200 ( .A(reshi_0_), .Z(n304) );
  HS65_GS_IVX9 U201 ( .A(result_nxt[16]), .Z(n53) );
  HS65_GS_NAND2X7 U202 ( .A(n74), .B(n75), .Z(per_dout[15]) );
  HS65_GS_AOI222X2 U203 ( .A(op2[15]), .B(n7), .C(op1[15]), .D(n40), .E(
        result_nxt[15]), .F(n8), .Z(n74) );
  HS65_GS_AOI212X4 U204 ( .A(n33), .B(result_nxt[31]), .C(n34), .D(n324), .E(
        n76), .Z(n75) );
  HS65_GS_NAND4ABX3 U205 ( .A(per_addr[13]), .B(per_addr[12]), .C(n214), .D(
        n215), .Z(n181) );
  HS65_GS_NOR4X4 U206 ( .A(per_addr[5]), .B(per_addr[9]), .C(per_addr[6]), .D(
        per_addr[8]), .Z(n215) );
  HS65_GS_OA12X9 U207 ( .A(per_we[0]), .B(per_we[1]), .C(n118), .Z(n214) );
  HS65_GS_OA12X9 U208 ( .A(cycle[1]), .B(cycle[0]), .C(n179), .Z(n180) );
  HS65_GS_OAI22X6 U209 ( .A(n242), .B(n203), .C(n136), .D(n139), .Z(n278) );
  HS65_GS_OAI22X6 U210 ( .A(n203), .B(n170), .C(n159), .D(n207), .Z(n296) );
  HS65_GS_IVX9 U211 ( .A(op1[8]), .Z(n170) );
  HS65_GS_OAI22X6 U212 ( .A(n203), .B(n169), .C(n160), .D(n207), .Z(n297) );
  HS65_GS_IVX9 U213 ( .A(op1[9]), .Z(n169) );
  HS65_GS_OAI22X6 U214 ( .A(n203), .B(n167), .C(n141), .D(n207), .Z(n298) );
  HS65_GS_IVX9 U215 ( .A(op1[10]), .Z(n167) );
  HS65_GS_OAI22X6 U216 ( .A(n203), .B(n166), .C(n142), .D(n207), .Z(n299) );
  HS65_GS_IVX9 U217 ( .A(op1[11]), .Z(n166) );
  HS65_GS_OAI22X6 U218 ( .A(n203), .B(n165), .C(n143), .D(n207), .Z(n300) );
  HS65_GS_IVX9 U219 ( .A(op1[12]), .Z(n165) );
  HS65_GS_OAI22X6 U220 ( .A(n203), .B(n164), .C(n144), .D(n207), .Z(n301) );
  HS65_GS_IVX9 U221 ( .A(op1[13]), .Z(n164) );
  HS65_GS_OAI22X6 U222 ( .A(n203), .B(n163), .C(n145), .D(n207), .Z(n302) );
  HS65_GS_IVX9 U223 ( .A(op1[14]), .Z(n163) );
  HS65_GS_NOR4ABX2 U224 ( .A(per_addr[3]), .B(n216), .C(per_addr[11]), .D(
        per_addr[10]), .Z(n118) );
  HS65_GS_AND3X9 U225 ( .A(per_addr[7]), .B(per_addr[4]), .C(per_en), .Z(n216)
         );
  HS65_GS_NAND2X7 U226 ( .A(n242), .B(n132), .Z(n179) );
  HS65_GS_NAND2X7 U227 ( .A(n103), .B(n104), .Z(per_dout[0]) );
  HS65_GS_AOI222X2 U228 ( .A(op2[0]), .B(n7), .C(op1[0]), .D(n40), .E(
        result_nxt[0]), .F(n8), .Z(n103) );
  HS65_GS_AOI212X4 U229 ( .A(result_nxt[16]), .B(n33), .C(reshi_0_), .D(n34), 
        .E(n105), .Z(n104) );
  HS65_GS_OAI32X5 U230 ( .A(n133), .B(n107), .C(n135), .D(n232), .E(n36), .Z(
        n105) );
  HS65_GS_NAND2X7 U231 ( .A(n70), .B(n71), .Z(per_dout[1]) );
  HS65_GS_AOI222X2 U232 ( .A(op2[1]), .B(n7), .C(op1[1]), .D(n40), .E(
        result_nxt[1]), .F(n8), .Z(n70) );
  HS65_GS_AOI212X4 U233 ( .A(result_nxt[17]), .B(n33), .C(n34), .D(n310), .E(
        n72), .Z(n71) );
  HS65_GS_NAND2X7 U234 ( .A(n66), .B(n67), .Z(per_dout[2]) );
  HS65_GS_AOI222X2 U235 ( .A(op2[2]), .B(n7), .C(op1[2]), .D(n40), .E(
        result_nxt[2]), .F(n8), .Z(n66) );
  HS65_GS_AOI212X4 U236 ( .A(result_nxt[18]), .B(n33), .C(n34), .D(n311), .E(
        n68), .Z(n67) );
  HS65_GS_NAND2X7 U237 ( .A(n62), .B(n63), .Z(per_dout[3]) );
  HS65_GS_AOI222X2 U238 ( .A(op2[3]), .B(n7), .C(op1[3]), .D(n40), .E(
        result_nxt[3]), .F(n8), .Z(n62) );
  HS65_GS_AOI212X4 U239 ( .A(result_nxt[19]), .B(n33), .C(n34), .D(n312), .E(
        n64), .Z(n63) );
  HS65_GS_NAND2X7 U240 ( .A(n58), .B(n59), .Z(per_dout[4]) );
  HS65_GS_AOI222X2 U241 ( .A(op2[4]), .B(n7), .C(op1[4]), .D(n40), .E(
        result_nxt[4]), .F(n8), .Z(n58) );
  HS65_GS_AOI212X4 U242 ( .A(result_nxt[20]), .B(n33), .C(n34), .D(n313), .E(
        n60), .Z(n59) );
  HS65_GS_NAND2X7 U243 ( .A(n54), .B(n55), .Z(per_dout[5]) );
  HS65_GS_AOI222X2 U244 ( .A(op2[5]), .B(n7), .C(op1[5]), .D(n40), .E(
        result_nxt[5]), .F(n8), .Z(n54) );
  HS65_GS_AOI212X4 U245 ( .A(result_nxt[21]), .B(n33), .C(n34), .D(n314), .E(
        n56), .Z(n55) );
  HS65_GS_NAND2X7 U246 ( .A(n50), .B(n51), .Z(per_dout[6]) );
  HS65_GS_AOI222X2 U247 ( .A(op2[6]), .B(n7), .C(op1[6]), .D(n40), .E(
        result_nxt[6]), .F(n8), .Z(n50) );
  HS65_GS_AOI212X4 U248 ( .A(result_nxt[22]), .B(n33), .C(n34), .D(n315), .E(
        n52), .Z(n51) );
  HS65_GS_NAND2X7 U249 ( .A(n46), .B(n47), .Z(per_dout[7]) );
  HS65_GS_AOI222X2 U250 ( .A(op2[7]), .B(n7), .C(op1[7]), .D(n40), .E(
        result_nxt[7]), .F(n8), .Z(n46) );
  HS65_GS_AOI212X4 U251 ( .A(result_nxt[23]), .B(n33), .C(n34), .D(n316), .E(
        n48), .Z(n47) );
  HS65_GS_NAND2X7 U252 ( .A(n42), .B(n43), .Z(per_dout[8]) );
  HS65_GS_AOI222X2 U253 ( .A(op2[8]), .B(n7), .C(op1[8]), .D(n40), .E(
        result_nxt[8]), .F(n8), .Z(n42) );
  HS65_GS_AOI212X4 U254 ( .A(result_nxt[24]), .B(n33), .C(n34), .D(n317), .E(
        n44), .Z(n43) );
  HS65_GS_NAND2X7 U255 ( .A(n31), .B(n32), .Z(per_dout[9]) );
  HS65_GS_AOI222X2 U256 ( .A(op2[9]), .B(n7), .C(op1[9]), .D(n40), .E(
        result_nxt[9]), .F(n8), .Z(n31) );
  HS65_GS_AOI212X4 U257 ( .A(result_nxt[25]), .B(n33), .C(n34), .D(n318), .E(
        n35), .Z(n32) );
  HS65_GS_NAND2X7 U258 ( .A(n94), .B(n95), .Z(per_dout[10]) );
  HS65_GS_AOI222X2 U259 ( .A(op2[10]), .B(n7), .C(op1[10]), .D(n40), .E(
        result_nxt[10]), .F(n8), .Z(n94) );
  HS65_GS_AOI212X4 U260 ( .A(result_nxt[26]), .B(n33), .C(n34), .D(n319), .E(
        n96), .Z(n95) );
  HS65_GS_NAND2X7 U261 ( .A(n90), .B(n91), .Z(per_dout[11]) );
  HS65_GS_AOI222X2 U262 ( .A(op2[11]), .B(n7), .C(op1[11]), .D(n40), .E(
        result_nxt[11]), .F(n8), .Z(n90) );
  HS65_GS_AOI212X4 U263 ( .A(result_nxt[27]), .B(n33), .C(n34), .D(n320), .E(
        n92), .Z(n91) );
  HS65_GS_NAND2X7 U264 ( .A(n86), .B(n87), .Z(per_dout[12]) );
  HS65_GS_AOI222X2 U265 ( .A(op2[12]), .B(n7), .C(op1[12]), .D(n40), .E(
        result_nxt[12]), .F(n8), .Z(n86) );
  HS65_GS_AOI212X4 U266 ( .A(result_nxt[28]), .B(n33), .C(n34), .D(n321), .E(
        n88), .Z(n87) );
  HS65_GS_NAND2X7 U267 ( .A(n82), .B(n83), .Z(per_dout[13]) );
  HS65_GS_AOI222X2 U268 ( .A(op2[13]), .B(n7), .C(op1[13]), .D(n40), .E(
        result_nxt[13]), .F(n8), .Z(n82) );
  HS65_GS_AOI212X4 U269 ( .A(result_nxt[29]), .B(n33), .C(n34), .D(n322), .E(
        n84), .Z(n83) );
  HS65_GS_NAND2X7 U270 ( .A(n78), .B(n79), .Z(per_dout[14]) );
  HS65_GS_AOI222X2 U271 ( .A(op2[14]), .B(n7), .C(op1[14]), .D(n40), .E(
        result_nxt[14]), .F(n8), .Z(n78) );
  HS65_GS_AOI212X4 U272 ( .A(result_nxt[30]), .B(n33), .C(n34), .D(n323), .E(
        n80), .Z(n79) );
  HS65_GS_AO22X9 U273 ( .A(n139), .B(op1[0]), .C(per_din[0]), .D(n203), .Z(
        n288) );
  HS65_GS_AO22X9 U274 ( .A(n139), .B(op1[1]), .C(per_din[1]), .D(n203), .Z(
        n289) );
  HS65_GS_AO22X9 U275 ( .A(n139), .B(op1[2]), .C(per_din[2]), .D(n203), .Z(
        n290) );
  HS65_GS_AO22X9 U276 ( .A(n139), .B(op1[3]), .C(per_din[3]), .D(n203), .Z(
        n291) );
  HS65_GS_AO22X9 U277 ( .A(n139), .B(op1[4]), .C(per_din[4]), .D(n203), .Z(
        n292) );
  HS65_GS_AO22X9 U278 ( .A(n139), .B(op1[5]), .C(per_din[5]), .D(n203), .Z(
        n293) );
  HS65_GS_AO22X9 U279 ( .A(n139), .B(op1[6]), .C(per_din[6]), .D(n203), .Z(
        n294) );
  HS65_GS_AO22X9 U280 ( .A(n139), .B(op1[7]), .C(per_din[7]), .D(n203), .Z(
        n295) );
  HS65_GS_NAND3X5 U281 ( .A(product[23]), .B(n10), .C(sign_sel), .Z(n17) );
  HS65_GS_IVX9 U282 ( .A(sign_sel), .Z(n195) );
  HS65_GS_OAI22X6 U283 ( .A(n11), .B(n171), .C(n9), .D(n112), .Z(product_xp[8]) );
  HS65_GS_IVX9 U284 ( .A(n226), .Z(n315) );
  HS65_GS_OAI22X6 U285 ( .A(n11), .B(n73), .C(cycle[0]), .D(n25), .Z(
        product_xp[22]) );
  HS65_GS_IVX9 U286 ( .A(n231), .Z(n310) );
  HS65_GS_OAI22X6 U287 ( .A(n106), .B(n11), .C(cycle[0]), .D(n49), .Z(
        product_xp[17]) );
  HS65_GS_OAI22X6 U288 ( .A(n10), .B(n130), .C(n9), .D(n106), .Z(product_xp[9]) );
  HS65_GS_OAI22X6 U289 ( .A(n11), .B(n124), .C(n9), .D(n89), .Z(product_xp[12]) );
  HS65_GS_CBI4I6X5 U290 ( .A(sumext_s[0]), .B(result_nxt[32]), .C(n195), .D(
        n148), .Z(n111) );
  HS65_GS_IVX9 U291 ( .A(n217), .Z(n324) );
  HS65_GS_AOI12X2 U292 ( .A(n195), .B(n10), .C(n23), .Z(product_xp[31]) );
  HS65_GS_OAI22X6 U293 ( .A(n11), .B(n112), .C(cycle[0]), .D(n57), .Z(
        product_xp[16]) );
  HS65_GS_NOR2X6 U294 ( .A(cycle[0]), .B(n130), .Z(product_xp[1]) );
  HS65_GS_NOR2X6 U295 ( .A(cycle[0]), .B(n128), .Z(product_xp[2]) );
  HS65_GS_NOR2X6 U296 ( .A(n9), .B(n126), .Z(product_xp[3]) );
  HS65_GS_NOR2X6 U297 ( .A(n9), .B(n124), .Z(product_xp[4]) );
  HS65_GS_NOR2X6 U298 ( .A(n9), .B(n122), .Z(product_xp[5]) );
  HS65_GS_NOR2X6 U299 ( .A(n9), .B(n114), .Z(product_xp[7]) );
  HS65_GS_OAI22X6 U300 ( .A(n11), .B(n128), .C(n9), .D(n101), .Z(
        product_xp[10]) );
  HS65_GS_OAI22X6 U301 ( .A(n11), .B(n126), .C(n9), .D(n97), .Z(product_xp[11]) );
  HS65_GS_OAI22X6 U302 ( .A(n11), .B(n122), .C(n9), .D(n81), .Z(product_xp[13]) );
  HS65_GS_OAI22X6 U303 ( .A(n11), .B(n120), .C(n9), .D(n73), .Z(product_xp[14]) );
  HS65_GS_OAI22X6 U304 ( .A(n11), .B(n114), .C(n9), .D(n65), .Z(product_xp[15]) );
  HS65_GS_IVX9 U305 ( .A(n230), .Z(n311) );
  HS65_GS_OAI22X6 U306 ( .A(n11), .B(n101), .C(cycle[0]), .D(n41), .Z(
        product_xp[18]) );
  HS65_GS_IVX9 U307 ( .A(n229), .Z(n312) );
  HS65_GS_OAI22X6 U308 ( .A(n11), .B(n97), .C(cycle[0]), .D(n37), .Z(
        product_xp[19]) );
  HS65_GS_IVX9 U309 ( .A(n228), .Z(n313) );
  HS65_GS_OAI22X6 U310 ( .A(n11), .B(n89), .C(cycle[0]), .D(n29), .Z(
        product_xp[20]) );
  HS65_GS_IVX9 U311 ( .A(n227), .Z(n314) );
  HS65_GS_OAI22X6 U312 ( .A(n11), .B(n81), .C(cycle[0]), .D(n27), .Z(
        product_xp[21]) );
  HS65_GS_IVX9 U313 ( .A(n225), .Z(n316) );
  HS65_GS_OAI22X6 U314 ( .A(n11), .B(n65), .C(cycle[0]), .D(n23), .Z(
        product_xp[23]) );
  HS65_GS_IVX9 U315 ( .A(n224), .Z(n317) );
  HS65_GS_OAI21X3 U316 ( .A(n10), .B(n57), .C(n17), .Z(product_xp[24]) );
  HS65_GS_IVX9 U317 ( .A(n223), .Z(n318) );
  HS65_GS_OAI21X3 U318 ( .A(n10), .B(n49), .C(n17), .Z(product_xp[25]) );
  HS65_GS_IVX9 U319 ( .A(n222), .Z(n319) );
  HS65_GS_OAI21X3 U320 ( .A(n10), .B(n41), .C(n17), .Z(product_xp[26]) );
  HS65_GS_IVX9 U321 ( .A(n221), .Z(n320) );
  HS65_GS_OAI21X3 U322 ( .A(n10), .B(n37), .C(n17), .Z(product_xp[27]) );
  HS65_GS_IVX9 U323 ( .A(n220), .Z(n321) );
  HS65_GS_OAI21X3 U324 ( .A(n10), .B(n29), .C(n17), .Z(product_xp[28]) );
  HS65_GS_IVX9 U325 ( .A(n219), .Z(n322) );
  HS65_GS_OAI21X3 U326 ( .A(n10), .B(n27), .C(n17), .Z(product_xp[29]) );
  HS65_GS_IVX9 U327 ( .A(n218), .Z(n323) );
  HS65_GS_OAI21X3 U328 ( .A(n10), .B(n25), .C(n17), .Z(product_xp[30]) );
  HS65_GS_IVX9 U329 ( .A(op2[15]), .Z(n172) );
  HS65_GS_NOR2X6 U330 ( .A(n9), .B(n120), .Z(product_xp[6]) );
  HS65_GS_BFX9 U331 ( .A(cycle[0]), .Z(n9) );
  HS65_GS_IVX9 U332 ( .A(op2[0]), .Z(n194) );
  HS65_GS_IVX9 U333 ( .A(op2[1]), .Z(n192) );
  HS65_GS_IVX9 U334 ( .A(op2[2]), .Z(n191) );
  HS65_GS_IVX9 U335 ( .A(op2[3]), .Z(n190) );
  HS65_GS_IVX9 U336 ( .A(op2[4]), .Z(n189) );
  HS65_GS_IVX9 U337 ( .A(op2[7]), .Z(n186) );
  HS65_GS_IVX9 U338 ( .A(op2[6]), .Z(n187) );
  HS65_GS_IVX9 U339 ( .A(op2[5]), .Z(n188) );
  HS65_GS_IVX9 U340 ( .A(op2[8]), .Z(n184) );
  HS65_GS_IVX9 U341 ( .A(op2[9]), .Z(n178) );
  HS65_GS_IVX9 U342 ( .A(op2[10]), .Z(n177) );
  HS65_GS_IVX9 U343 ( .A(op2[11]), .Z(n176) );
  HS65_GS_IVX9 U344 ( .A(op2[12]), .Z(n175) );
  HS65_GS_IVX9 U345 ( .A(op2[14]), .Z(n173) );
  HS65_GS_IVX9 U346 ( .A(op2[13]), .Z(n174) );
  HS65_GS_IVX9 U347 ( .A(n232), .Z(n309) );
  HS65_GS_IVX9 U348 ( .A(op1[15]), .Z(n162) );
  HS65_GS_IVX9 U349 ( .A(cycle[1]), .Z(n196) );
  HS65_GS_IVX9 U350 ( .A(reslo[1]), .Z(n197) );
  HS65_GS_IVX9 U351 ( .A(reslo[2]), .Z(n198) );
  HS65_GS_IVX9 U352 ( .A(reslo[3]), .Z(n199) );
  HS65_GS_IVX9 U353 ( .A(reslo[4]), .Z(n200) );
  HS65_GS_IVX9 U354 ( .A(reslo[5]), .Z(n201) );
  HS65_GS_IVX9 U355 ( .A(reslo[6]), .Z(n204) );
  HS65_GS_IVX9 U356 ( .A(reslo[7]), .Z(n206) );
  HS65_GS_IVX9 U357 ( .A(reslo[8]), .Z(n208) );
  HS65_GS_IVX9 U358 ( .A(reslo[9]), .Z(n209) );
  HS65_GS_IVX9 U359 ( .A(reslo[10]), .Z(n210) );
  HS65_GS_IVX9 U360 ( .A(reslo[11]), .Z(n211) );
  HS65_GS_IVX9 U361 ( .A(reslo[12]), .Z(n212) );
  HS65_GS_IVX9 U362 ( .A(reslo[13]), .Z(n213) );
  HS65_GS_IVX9 U363 ( .A(reslo[14]), .Z(n241) );
  HS65_GS_IVX9 U364 ( .A(reslo[15]), .Z(n243) );
  HS65_GS_IVX9 U365 ( .A(sumext_s[0]), .Z(n306) );
  HS65_GS_IVX9 U366 ( .A(sumext_s[1]), .Z(n305) );
endmodule


module openMSP430 ( aclk, aclk_en, dbg_freeze, dbg_i2c_sda_out, dbg_uart_txd, 
        dco_enable, dco_wkup, dmem_addr, dmem_cen, dmem_din, dmem_wen, irq_acc, 
        lfxt_enable, lfxt_wkup, mclk, dma_dout, dma_ready, dma_resp, per_addr, 
        per_din, per_en, per_we, pmem_addr, pmem_cen, pmem_din, pmem_wen, 
        puc_rst, smclk, smclk_en, cpu_en, dbg_en, dbg_i2c_addr, 
        dbg_i2c_broadcast, dbg_i2c_scl, dbg_i2c_sda_in, dbg_uart_rxd, dco_clk, 
        dmem_dout, irq, lfxt_clk, dma_addr, dma_din, dma_en, dma_priority, 
        dma_we, dma_wkup, nmi, per_dout, pmem_dout, reset_n, scan_enable, 
        scan_mode, wkup );
  output [12:0] dmem_addr;
  output [15:0] dmem_din;
  output [1:0] dmem_wen;
  output [13:0] irq_acc;
  output [15:0] dma_dout;
  output [13:0] per_addr;
  output [15:0] per_din;
  output [1:0] per_we;
  output [14:0] pmem_addr;
  output [15:0] pmem_din;
  output [1:0] pmem_wen;
  input [6:0] dbg_i2c_addr;
  input [6:0] dbg_i2c_broadcast;
  input [15:0] dmem_dout;
  input [13:0] irq;
  input [15:1] dma_addr;
  input [15:0] dma_din;
  input [1:0] dma_we;
  input [15:0] per_dout;
  input [15:0] pmem_dout;
  input cpu_en, dbg_en, dbg_i2c_scl, dbg_i2c_sda_in, dbg_uart_rxd, dco_clk,
         lfxt_clk, dma_en, dma_priority, dma_wkup, nmi, reset_n, scan_enable,
         scan_mode, wkup;
  output aclk, aclk_en, dbg_freeze, dbg_i2c_sda_out, dbg_uart_txd, dco_enable,
         dco_wkup, dmem_cen, lfxt_enable, lfxt_wkup, mclk, dma_ready, dma_resp,
         per_en, pmem_cen, puc_rst, smclk, smclk_en;
  wire   cpu_en_s, cpu_mclk, cpu_halt_st, exec_done, inst_bw, inst_irq_rst,
         inst_mov, fe_mb_en, nmi_acc, cpu_halt_cmd, fe_pmem_wait, gie,
         pc_sw_wr, eu_mb_en, n66, n68, SYNOPSYS_UNCONNECTED_1,
         SYNOPSYS_UNCONNECTED_2, SYNOPSYS_UNCONNECTED_3,
         SYNOPSYS_UNCONNECTED_4, SYNOPSYS_UNCONNECTED_5,
         SYNOPSYS_UNCONNECTED_6, SYNOPSYS_UNCONNECTED_7,
         SYNOPSYS_UNCONNECTED_8, SYNOPSYS_UNCONNECTED_9,
         SYNOPSYS_UNCONNECTED_10, SYNOPSYS_UNCONNECTED_11,
         SYNOPSYS_UNCONNECTED_12, SYNOPSYS_UNCONNECTED_13,
         SYNOPSYS_UNCONNECTED_14, SYNOPSYS_UNCONNECTED_15,
         SYNOPSYS_UNCONNECTED_16, SYNOPSYS_UNCONNECTED_17,
         SYNOPSYS_UNCONNECTED_18, SYNOPSYS_UNCONNECTED_19,
         SYNOPSYS_UNCONNECTED_20, SYNOPSYS_UNCONNECTED_21,
         SYNOPSYS_UNCONNECTED_22, SYNOPSYS_UNCONNECTED_23,
         SYNOPSYS_UNCONNECTED_24, SYNOPSYS_UNCONNECTED_25,
         SYNOPSYS_UNCONNECTED_26, SYNOPSYS_UNCONNECTED_27,
         SYNOPSYS_UNCONNECTED_28, SYNOPSYS_UNCONNECTED_29,
         SYNOPSYS_UNCONNECTED_30, SYNOPSYS_UNCONNECTED_31,
         SYNOPSYS_UNCONNECTED_32, SYNOPSYS_UNCONNECTED_33,
         SYNOPSYS_UNCONNECTED_34, SYNOPSYS_UNCONNECTED_35,
         SYNOPSYS_UNCONNECTED_36, SYNOPSYS_UNCONNECTED_37,
         SYNOPSYS_UNCONNECTED_38, SYNOPSYS_UNCONNECTED_39,
         SYNOPSYS_UNCONNECTED_40, SYNOPSYS_UNCONNECTED_41,
         SYNOPSYS_UNCONNECTED_42, SYNOPSYS_UNCONNECTED_43,
         SYNOPSYS_UNCONNECTED_44, SYNOPSYS_UNCONNECTED_45,
         SYNOPSYS_UNCONNECTED_46, SYNOPSYS_UNCONNECTED_47,
         SYNOPSYS_UNCONNECTED_48, SYNOPSYS_UNCONNECTED_49,
         SYNOPSYS_UNCONNECTED_50, SYNOPSYS_UNCONNECTED_51,
         SYNOPSYS_UNCONNECTED_52, SYNOPSYS_UNCONNECTED_53,
         SYNOPSYS_UNCONNECTED_54, SYNOPSYS_UNCONNECTED_55,
         SYNOPSYS_UNCONNECTED_56, SYNOPSYS_UNCONNECTED_57,
         SYNOPSYS_UNCONNECTED_58, SYNOPSYS_UNCONNECTED_59,
         SYNOPSYS_UNCONNECTED_60, SYNOPSYS_UNCONNECTED_61,
         SYNOPSYS_UNCONNECTED_62, SYNOPSYS_UNCONNECTED_63,
         SYNOPSYS_UNCONNECTED_64, SYNOPSYS_UNCONNECTED_65,
         SYNOPSYS_UNCONNECTED_66, SYNOPSYS_UNCONNECTED_67,
         SYNOPSYS_UNCONNECTED_68, SYNOPSYS_UNCONNECTED_69,
         SYNOPSYS_UNCONNECTED_70, SYNOPSYS_UNCONNECTED_71,
         SYNOPSYS_UNCONNECTED_72, SYNOPSYS_UNCONNECTED_73,
         SYNOPSYS_UNCONNECTED_74, SYNOPSYS_UNCONNECTED_75,
         SYNOPSYS_UNCONNECTED_76, SYNOPSYS_UNCONNECTED_77,
         SYNOPSYS_UNCONNECTED_78, SYNOPSYS_UNCONNECTED_79,
         SYNOPSYS_UNCONNECTED_80, SYNOPSYS_UNCONNECTED_81,
         SYNOPSYS_UNCONNECTED_82, SYNOPSYS_UNCONNECTED_83,
         SYNOPSYS_UNCONNECTED_84, SYNOPSYS_UNCONNECTED_85,
         SYNOPSYS_UNCONNECTED_86, SYNOPSYS_UNCONNECTED_87,
         SYNOPSYS_UNCONNECTED_88, SYNOPSYS_UNCONNECTED_89,
         SYNOPSYS_UNCONNECTED_90, SYNOPSYS_UNCONNECTED_91,
         SYNOPSYS_UNCONNECTED_92, SYNOPSYS_UNCONNECTED_93,
         SYNOPSYS_UNCONNECTED_94, SYNOPSYS_UNCONNECTED_95,
         SYNOPSYS_UNCONNECTED_96, SYNOPSYS_UNCONNECTED_97,
         SYNOPSYS_UNCONNECTED_98, SYNOPSYS_UNCONNECTED_99,
         SYNOPSYS_UNCONNECTED_100, SYNOPSYS_UNCONNECTED_101,
         SYNOPSYS_UNCONNECTED_102, SYNOPSYS_UNCONNECTED_103,
         SYNOPSYS_UNCONNECTED_104, SYNOPSYS_UNCONNECTED_105,
         SYNOPSYS_UNCONNECTED_106, SYNOPSYS_UNCONNECTED_107,
         SYNOPSYS_UNCONNECTED_108, SYNOPSYS_UNCONNECTED_109,
         SYNOPSYS_UNCONNECTED_110, SYNOPSYS_UNCONNECTED_111,
         SYNOPSYS_UNCONNECTED_112, SYNOPSYS_UNCONNECTED_113,
         SYNOPSYS_UNCONNECTED_114, SYNOPSYS_UNCONNECTED_115,
         SYNOPSYS_UNCONNECTED_116, SYNOPSYS_UNCONNECTED_117,
         SYNOPSYS_UNCONNECTED_118, SYNOPSYS_UNCONNECTED_119,
         SYNOPSYS_UNCONNECTED_120, SYNOPSYS_UNCONNECTED_121,
         SYNOPSYS_UNCONNECTED_122, SYNOPSYS_UNCONNECTED_123,
         SYNOPSYS_UNCONNECTED_124, SYNOPSYS_UNCONNECTED_125,
         SYNOPSYS_UNCONNECTED_126, SYNOPSYS_UNCONNECTED_127,
         SYNOPSYS_UNCONNECTED_128, SYNOPSYS_UNCONNECTED_129,
         SYNOPSYS_UNCONNECTED_130, SYNOPSYS_UNCONNECTED_131,
         SYNOPSYS_UNCONNECTED_132, SYNOPSYS_UNCONNECTED_133,
         SYNOPSYS_UNCONNECTED_134, SYNOPSYS_UNCONNECTED_135,
         SYNOPSYS_UNCONNECTED_136, SYNOPSYS_UNCONNECTED_137,
         SYNOPSYS_UNCONNECTED_138, SYNOPSYS_UNCONNECTED_139,
         SYNOPSYS_UNCONNECTED_140, SYNOPSYS_UNCONNECTED_141,
         SYNOPSYS_UNCONNECTED_142, SYNOPSYS_UNCONNECTED_143,
         SYNOPSYS_UNCONNECTED_144, SYNOPSYS_UNCONNECTED_145,
         SYNOPSYS_UNCONNECTED_146, SYNOPSYS_UNCONNECTED_147,
         SYNOPSYS_UNCONNECTED_148, SYNOPSYS_UNCONNECTED_149,
         SYNOPSYS_UNCONNECTED_150, SYNOPSYS_UNCONNECTED_151;
  wire   [3:0] e_state;
  wire   [7:0] inst_ad;
  wire   [7:0] inst_as;
  wire   [11:0] inst_alu;
  wire   [15:0] inst_dest;
  wire   [15:0] inst_dext;
  wire   [7:0] inst_jmp;
  wire   [15:0] inst_sext;
  wire   [7:0] inst_so;
  wire   [15:0] inst_src;
  wire   [2:0] inst_type;
  wire   [15:1] fe_mab;
  wire   [15:0] pc;
  wire   [15:0] pc_nxt;
  wire   [15:0] fe_mdb_in;
  wire   [15:0] pc_sw;
  wire   [15:1] eu_mab;
  wire   [1:0] eu_mb_wr;
  wire   [15:0] eu_mdb_out;
  wire   [15:0] eu_mdb_in;
  wire   [15:0] per_dout_or;
  wire   [15:0] per_dout_sfr;
  wire   [15:0] per_dout_mpy;

  omsp_clock_module clock_module_0 ( .aclk(aclk), .cpu_en_s(cpu_en_s), 
        .cpu_mclk(cpu_mclk), .dma_mclk(mclk), .per_dout({
        SYNOPSYS_UNCONNECTED_1, SYNOPSYS_UNCONNECTED_2, SYNOPSYS_UNCONNECTED_3, 
        SYNOPSYS_UNCONNECTED_4, SYNOPSYS_UNCONNECTED_5, SYNOPSYS_UNCONNECTED_6, 
        SYNOPSYS_UNCONNECTED_7, SYNOPSYS_UNCONNECTED_8, SYNOPSYS_UNCONNECTED_9, 
        SYNOPSYS_UNCONNECTED_10, SYNOPSYS_UNCONNECTED_11, 
        SYNOPSYS_UNCONNECTED_12, SYNOPSYS_UNCONNECTED_13, 
        SYNOPSYS_UNCONNECTED_14, SYNOPSYS_UNCONNECTED_15, 
        SYNOPSYS_UNCONNECTED_16}), .puc_rst(puc_rst), .smclk(smclk), .cpu_en(
        cpu_en), .cpuoff(1'b0), .dbg_cpu_reset(1'b0), .dbg_en(dbg_en), 
        .dco_clk(dco_clk), .lfxt_clk(lfxt_clk), .mclk_dma_enable(1'b1), 
        .mclk_dma_wkup(1'b1), .mclk_enable(1'b1), .mclk_wkup(1'b1), .oscoff(
        1'b0), .per_addr({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, per_addr[7:0]}), 
        .per_din(per_din), .per_en(per_en), .per_we(per_we), .reset_n(reset_n), 
        .scan_enable(scan_enable), .scan_mode(scan_mode), .scg0(1'b0), .scg1(
        1'b0), .wdt_reset(1'b0) );
  omsp_frontend frontend_0 ( .OBSERVE_i_state({SYNOPSYS_UNCONNECTED_17, 
        SYNOPSYS_UNCONNECTED_18, SYNOPSYS_UNCONNECTED_19}), .OBSERVE_e_state({
        SYNOPSYS_UNCONNECTED_20, SYNOPSYS_UNCONNECTED_21, 
        SYNOPSYS_UNCONNECTED_22, SYNOPSYS_UNCONNECTED_23}), .OBSERVE_ir({
        SYNOPSYS_UNCONNECTED_24, SYNOPSYS_UNCONNECTED_25, 
        SYNOPSYS_UNCONNECTED_26, SYNOPSYS_UNCONNECTED_27, 
        SYNOPSYS_UNCONNECTED_28, SYNOPSYS_UNCONNECTED_29, 
        SYNOPSYS_UNCONNECTED_30, SYNOPSYS_UNCONNECTED_31, 
        SYNOPSYS_UNCONNECTED_32, SYNOPSYS_UNCONNECTED_33, 
        SYNOPSYS_UNCONNECTED_34, SYNOPSYS_UNCONNECTED_35, 
        SYNOPSYS_UNCONNECTED_36, SYNOPSYS_UNCONNECTED_37, 
        SYNOPSYS_UNCONNECTED_38, SYNOPSYS_UNCONNECTED_39}), .OBSERVE_irq_num({
        SYNOPSYS_UNCONNECTED_40, SYNOPSYS_UNCONNECTED_41, 
        SYNOPSYS_UNCONNECTED_42, SYNOPSYS_UNCONNECTED_43}), .OBSERVE_pc({
        SYNOPSYS_UNCONNECTED_44, SYNOPSYS_UNCONNECTED_45, 
        SYNOPSYS_UNCONNECTED_46, SYNOPSYS_UNCONNECTED_47, 
        SYNOPSYS_UNCONNECTED_48, SYNOPSYS_UNCONNECTED_49, 
        SYNOPSYS_UNCONNECTED_50, SYNOPSYS_UNCONNECTED_51, 
        SYNOPSYS_UNCONNECTED_52, SYNOPSYS_UNCONNECTED_53, 
        SYNOPSYS_UNCONNECTED_54, SYNOPSYS_UNCONNECTED_55, 
        SYNOPSYS_UNCONNECTED_56, SYNOPSYS_UNCONNECTED_57, 
        SYNOPSYS_UNCONNECTED_58, SYNOPSYS_UNCONNECTED_59}), .cpu_halt_st(
        cpu_halt_st), .e_state(e_state), .exec_done(exec_done), .inst_ad({
        SYNOPSYS_UNCONNECTED_60, inst_ad[6], SYNOPSYS_UNCONNECTED_61, 
        inst_ad[4], SYNOPSYS_UNCONNECTED_62, SYNOPSYS_UNCONNECTED_63, 
        inst_ad[1:0]}), .inst_as(inst_as), .inst_alu(inst_alu), .inst_bw(
        inst_bw), .inst_dest(inst_dest), .inst_dext(inst_dext), .inst_irq_rst(
        inst_irq_rst), .inst_jmp(inst_jmp), .inst_mov(inst_mov), .inst_sext(
        inst_sext), .inst_so(inst_so), .inst_src(inst_src), .inst_type(
        inst_type), .irq_acc(irq_acc), .mab({fe_mab, SYNOPSYS_UNCONNECTED_64}), 
        .mb_en(fe_mb_en), .nmi_acc(nmi_acc), .pc(pc), .pc_nxt(pc_nxt), 
        .cpu_en_s(cpu_en_s), .cpu_halt_cmd(cpu_halt_cmd), .cpuoff(1'b0), 
        .dbg_reg_sel({1'b0, 1'b0, 1'b0, 1'b0}), .dma_en(dma_en), .dma_wkup(
        dma_wkup), .fe_pmem_wait(fe_pmem_wait), .gie(gie), .irq(irq), .mclk(
        cpu_mclk), .mdb_in(fe_mdb_in), .nmi_pnd(1'b0), .nmi_wkup(1'b0), 
        .pc_sw(pc_sw), .pc_sw_wr(n66), .puc_rst(puc_rst), .scan_enable(
        scan_enable), .wdt_irq(1'b0), .wdt_wkup(1'b0), .wkup(wkup) );
  omsp_execution_unit execution_unit_0 ( .dbg_reg_din({SYNOPSYS_UNCONNECTED_65, 
        SYNOPSYS_UNCONNECTED_66, SYNOPSYS_UNCONNECTED_67, 
        SYNOPSYS_UNCONNECTED_68, SYNOPSYS_UNCONNECTED_69, 
        SYNOPSYS_UNCONNECTED_70, SYNOPSYS_UNCONNECTED_71, 
        SYNOPSYS_UNCONNECTED_72, SYNOPSYS_UNCONNECTED_73, 
        SYNOPSYS_UNCONNECTED_74, SYNOPSYS_UNCONNECTED_75, 
        SYNOPSYS_UNCONNECTED_76, SYNOPSYS_UNCONNECTED_77, 
        SYNOPSYS_UNCONNECTED_78, SYNOPSYS_UNCONNECTED_79, 
        SYNOPSYS_UNCONNECTED_80}), .gie(gie), .mab({eu_mab, 
        SYNOPSYS_UNCONNECTED_81}), .mb_en(eu_mb_en), .mb_wr(eu_mb_wr), 
        .mdb_out(eu_mdb_out), .pc_sw(pc_sw), .pc_sw_wr(pc_sw_wr), 
        .dbg_halt_st(n68), .dbg_mem_dout({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), 
        .dbg_reg_wr(1'b0), .e_state(e_state), .exec_done(exec_done), .inst_ad(
        {1'b0, inst_ad[6], 1'b0, inst_ad[4], 1'b0, 1'b0, inst_ad[1:0]}), 
        .inst_as(inst_as), .inst_alu(inst_alu), .inst_bw(inst_bw), .inst_dest(
        inst_dest), .inst_dext(inst_dext), .inst_irq_rst(inst_irq_rst), 
        .inst_jmp(inst_jmp), .inst_mov(inst_mov), .inst_sext(inst_sext), 
        .inst_so(inst_so), .inst_src(inst_src), .inst_type(inst_type), .mclk(
        cpu_mclk), .mdb_in(eu_mdb_in), .pc(pc), .pc_nxt(pc_nxt), .puc_rst(
        puc_rst), .scan_enable(scan_enable) );
  omsp_mem_backbone mem_backbone_0 ( .cpu_halt_cmd(cpu_halt_cmd), 
        .dbg_mem_din({SYNOPSYS_UNCONNECTED_82, SYNOPSYS_UNCONNECTED_83, 
        SYNOPSYS_UNCONNECTED_84, SYNOPSYS_UNCONNECTED_85, 
        SYNOPSYS_UNCONNECTED_86, SYNOPSYS_UNCONNECTED_87, 
        SYNOPSYS_UNCONNECTED_88, SYNOPSYS_UNCONNECTED_89, 
        SYNOPSYS_UNCONNECTED_90, SYNOPSYS_UNCONNECTED_91, 
        SYNOPSYS_UNCONNECTED_92, SYNOPSYS_UNCONNECTED_93, 
        SYNOPSYS_UNCONNECTED_94, SYNOPSYS_UNCONNECTED_95, 
        SYNOPSYS_UNCONNECTED_96, SYNOPSYS_UNCONNECTED_97}), .dmem_addr(
        dmem_addr), .dmem_cen(dmem_cen), .dmem_din(dmem_din), .dmem_wen(
        dmem_wen), .eu_mdb_in(eu_mdb_in), .fe_mdb_in(fe_mdb_in), 
        .fe_pmem_wait(fe_pmem_wait), .dma_dout({SYNOPSYS_UNCONNECTED_98, 
        SYNOPSYS_UNCONNECTED_99, SYNOPSYS_UNCONNECTED_100, 
        SYNOPSYS_UNCONNECTED_101, SYNOPSYS_UNCONNECTED_102, 
        SYNOPSYS_UNCONNECTED_103, SYNOPSYS_UNCONNECTED_104, 
        SYNOPSYS_UNCONNECTED_105, SYNOPSYS_UNCONNECTED_106, 
        SYNOPSYS_UNCONNECTED_107, SYNOPSYS_UNCONNECTED_108, 
        SYNOPSYS_UNCONNECTED_109, SYNOPSYS_UNCONNECTED_110, 
        SYNOPSYS_UNCONNECTED_111, SYNOPSYS_UNCONNECTED_112, 
        SYNOPSYS_UNCONNECTED_113}), .per_addr({SYNOPSYS_UNCONNECTED_114, 
        SYNOPSYS_UNCONNECTED_115, SYNOPSYS_UNCONNECTED_116, 
        SYNOPSYS_UNCONNECTED_117, SYNOPSYS_UNCONNECTED_118, 
        SYNOPSYS_UNCONNECTED_119, per_addr[7:0]}), .per_din(per_din), .per_we(
        per_we), .per_en(per_en), .pmem_addr(pmem_addr), .pmem_cen(pmem_cen), 
        .pmem_din(pmem_din), .pmem_wen(pmem_wen), .cpu_halt_st(n68), 
        .dbg_halt_cmd(1'b0), .dbg_mem_addr({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), .dbg_mem_dout(
        {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), .dbg_mem_en(1'b0), .dbg_mem_wr({1'b0, 
        1'b0}), .dmem_dout(dmem_dout), .eu_mab(eu_mab), .eu_mb_en(eu_mb_en), 
        .eu_mb_wr(eu_mb_wr), .eu_mdb_out(eu_mdb_out), .fe_mab(fe_mab), 
        .fe_mb_en(fe_mb_en), .mclk(mclk), .dma_addr(dma_addr), .dma_din(
        dma_din), .dma_en(dma_en), .dma_priority(dma_priority), .dma_we(dma_we), .per_dout(per_dout_or), .pmem_dout(pmem_dout), .puc_rst(puc_rst), 
        .scan_enable(scan_enable) );
  omsp_sfr sfr_0 ( .cpu_id({SYNOPSYS_UNCONNECTED_120, SYNOPSYS_UNCONNECTED_121, 
        SYNOPSYS_UNCONNECTED_122, SYNOPSYS_UNCONNECTED_123, 
        SYNOPSYS_UNCONNECTED_124, SYNOPSYS_UNCONNECTED_125, 
        SYNOPSYS_UNCONNECTED_126, SYNOPSYS_UNCONNECTED_127, 
        SYNOPSYS_UNCONNECTED_128, SYNOPSYS_UNCONNECTED_129, 
        SYNOPSYS_UNCONNECTED_130, SYNOPSYS_UNCONNECTED_131, 
        SYNOPSYS_UNCONNECTED_132, SYNOPSYS_UNCONNECTED_133, 
        SYNOPSYS_UNCONNECTED_134, SYNOPSYS_UNCONNECTED_135, 
        SYNOPSYS_UNCONNECTED_136, SYNOPSYS_UNCONNECTED_137, 
        SYNOPSYS_UNCONNECTED_138, SYNOPSYS_UNCONNECTED_139, 
        SYNOPSYS_UNCONNECTED_140, SYNOPSYS_UNCONNECTED_141, 
        SYNOPSYS_UNCONNECTED_142, SYNOPSYS_UNCONNECTED_143, 
        SYNOPSYS_UNCONNECTED_144, SYNOPSYS_UNCONNECTED_145, 
        SYNOPSYS_UNCONNECTED_146, SYNOPSYS_UNCONNECTED_147, 
        SYNOPSYS_UNCONNECTED_148, SYNOPSYS_UNCONNECTED_149, 
        SYNOPSYS_UNCONNECTED_150, SYNOPSYS_UNCONNECTED_151}), .per_dout(
        per_dout_sfr), .cpu_nr_inst({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0}), .cpu_nr_total({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), .mclk(mclk), .nmi(nmi), .nmi_acc(nmi_acc), .per_addr({1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, per_addr[7:0]}), .per_din(per_din), .per_en(per_en), 
        .per_we(per_we), .puc_rst(puc_rst), .scan_mode(scan_mode), .wdtifg(
        1'b0), .wdtnmies(1'b0) );
  omsp_multiplier multiplier_0 ( .per_dout(per_dout_mpy), .mclk(mclk), 
        .per_addr({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, per_addr[7:0]}), 
        .per_din(per_din), .per_en(per_en), .per_we(per_we), .puc_rst(puc_rst), 
        .scan_enable(scan_enable) );
  HS65_GS_IVX9 U20 ( .A(1'b0), .Z(smclk_en) );
  HS65_GS_IVX9 U22 ( .A(1'b1), .Z(per_addr[8]) );
  HS65_GS_IVX9 U24 ( .A(1'b1), .Z(per_addr[9]) );
  HS65_GS_IVX9 U26 ( .A(1'b1), .Z(per_addr[10]) );
  HS65_GS_IVX9 U28 ( .A(1'b1), .Z(per_addr[11]) );
  HS65_GS_IVX9 U30 ( .A(1'b1), .Z(per_addr[12]) );
  HS65_GS_IVX9 U32 ( .A(1'b1), .Z(per_addr[13]) );
  HS65_GS_IVX9 U34 ( .A(1'b0), .Z(dma_resp) );
  HS65_GS_IVX9 U36 ( .A(1'b0), .Z(dma_ready) );
  HS65_GS_IVX9 U38 ( .A(1'b1), .Z(dma_dout[0]) );
  HS65_GS_IVX9 U40 ( .A(1'b1), .Z(dma_dout[1]) );
  HS65_GS_IVX9 U42 ( .A(1'b1), .Z(dma_dout[2]) );
  HS65_GS_IVX9 U44 ( .A(1'b1), .Z(dma_dout[3]) );
  HS65_GS_IVX9 U46 ( .A(1'b1), .Z(dma_dout[4]) );
  HS65_GS_IVX9 U48 ( .A(1'b1), .Z(dma_dout[5]) );
  HS65_GS_IVX9 U50 ( .A(1'b1), .Z(dma_dout[6]) );
  HS65_GS_IVX9 U52 ( .A(1'b1), .Z(dma_dout[7]) );
  HS65_GS_IVX9 U54 ( .A(1'b1), .Z(dma_dout[8]) );
  HS65_GS_IVX9 U56 ( .A(1'b1), .Z(dma_dout[9]) );
  HS65_GS_IVX9 U58 ( .A(1'b1), .Z(dma_dout[10]) );
  HS65_GS_IVX9 U60 ( .A(1'b1), .Z(dma_dout[11]) );
  HS65_GS_IVX9 U62 ( .A(1'b1), .Z(dma_dout[12]) );
  HS65_GS_IVX9 U64 ( .A(1'b1), .Z(dma_dout[13]) );
  HS65_GS_IVX9 U66 ( .A(1'b1), .Z(dma_dout[14]) );
  HS65_GS_IVX9 U68 ( .A(1'b1), .Z(dma_dout[15]) );
  HS65_GS_IVX9 U70 ( .A(1'b1), .Z(lfxt_wkup) );
  HS65_GS_IVX9 U72 ( .A(1'b0), .Z(lfxt_enable) );
  HS65_GS_IVX9 U74 ( .A(1'b0), .Z(dco_wkup) );
  HS65_GS_IVX9 U76 ( .A(1'b0), .Z(dco_enable) );
  HS65_GS_IVX9 U78 ( .A(1'b0), .Z(dbg_uart_txd) );
  HS65_GS_IVX9 U80 ( .A(1'b0), .Z(dbg_i2c_sda_out) );
  HS65_GS_IVX9 U82 ( .A(1'b0), .Z(aclk_en) );
  HS65_GS_BFX9 U84 ( .A(pc_sw_wr), .Z(n66) );
  HS65_GS_IVX9 U85 ( .A(cpu_en_s), .Z(dbg_freeze) );
  HS65_GS_BFX9 U86 ( .A(cpu_halt_st), .Z(n68) );
  HS65_GS_OR3X9 U87 ( .A(per_dout_sfr[8]), .B(per_dout[8]), .C(per_dout_mpy[8]), .Z(per_dout_or[8]) );
  HS65_GS_OR3X9 U88 ( .A(per_dout_sfr[9]), .B(per_dout[9]), .C(per_dout_mpy[9]), .Z(per_dout_or[9]) );
  HS65_GS_OR3X9 U89 ( .A(per_dout_sfr[10]), .B(per_dout[10]), .C(
        per_dout_mpy[10]), .Z(per_dout_or[10]) );
  HS65_GS_OR3X9 U90 ( .A(per_dout_sfr[11]), .B(per_dout[11]), .C(
        per_dout_mpy[11]), .Z(per_dout_or[11]) );
  HS65_GS_OR3X9 U91 ( .A(per_dout_sfr[12]), .B(per_dout[12]), .C(
        per_dout_mpy[12]), .Z(per_dout_or[12]) );
  HS65_GS_OR3X9 U92 ( .A(per_dout_sfr[13]), .B(per_dout[13]), .C(
        per_dout_mpy[13]), .Z(per_dout_or[13]) );
  HS65_GS_OR3X9 U93 ( .A(per_dout_sfr[14]), .B(per_dout[14]), .C(
        per_dout_mpy[14]), .Z(per_dout_or[14]) );
  HS65_GS_OR3X9 U94 ( .A(per_dout_sfr[15]), .B(per_dout[15]), .C(
        per_dout_mpy[15]), .Z(per_dout_or[15]) );
  HS65_GS_OR3X9 U95 ( .A(per_dout_sfr[0]), .B(per_dout[0]), .C(per_dout_mpy[0]), .Z(per_dout_or[0]) );
  HS65_GS_OR3X9 U96 ( .A(per_dout_sfr[1]), .B(per_dout[1]), .C(per_dout_mpy[1]), .Z(per_dout_or[1]) );
  HS65_GS_OR3X9 U97 ( .A(per_dout_sfr[2]), .B(per_dout[2]), .C(per_dout_mpy[2]), .Z(per_dout_or[2]) );
  HS65_GS_OR3X9 U98 ( .A(per_dout_sfr[3]), .B(per_dout[3]), .C(per_dout_mpy[3]), .Z(per_dout_or[3]) );
  HS65_GS_OR3X9 U99 ( .A(per_dout_sfr[4]), .B(per_dout[4]), .C(per_dout_mpy[4]), .Z(per_dout_or[4]) );
  HS65_GS_OR3X9 U100 ( .A(per_dout_sfr[5]), .B(per_dout[5]), .C(
        per_dout_mpy[5]), .Z(per_dout_or[5]) );
  HS65_GS_OR3X9 U101 ( .A(per_dout_sfr[6]), .B(per_dout[6]), .C(
        per_dout_mpy[6]), .Z(per_dout_or[6]) );
  HS65_GS_OR3X9 U102 ( .A(per_dout_sfr[7]), .B(per_dout[7]), .C(
        per_dout_mpy[7]), .Z(per_dout_or[7]) );
endmodule

