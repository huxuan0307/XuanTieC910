package Core.LSU.DCache
import chisel3._
import chisel3.util._

class DCacheDataArray extends Module {
  val io = IO(new Bundle{
    val data_din           = Input(UInt(32.W))
    val data_gateclk_en    = Input(Bool())
    val data_gwen_b        = Input(Bool())
    val data_idx           = Input(UInt(11.W))
    val data_sel_b         = Input(Bool())
    val data_wen_b         = Input(UInt(4.W))
    val pad_yy_icg_scan_en = Input(Bool())
    val cp0_lsu_icg_en     = Input(Bool())

    val data_dout = Output(UInt(32.W))
  })

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val data_clk_en = io.data_gateclk_en

  //==========================================================
  //              Instance dcache array
  //==========================================================
  val data_wen_b_all =
  Cat(VecInit(Seq.fill(8)(io.data_wen_b(3))).asUInt, VecInit(Seq.fill(8)(io.data_wen_b(2))).asUInt,
      VecInit(Seq.fill(8)(io.data_wen_b(1))).asUInt, VecInit(Seq.fill(8)(io.data_wen_b(0))).asUInt)

  val data_array = Module(new ct_spsram_2048x32)

  data_array.A    := io.data_idx
  data_array.CEN  := io.data_sel_b
  data_array.CLK  := clock
  data_array.D    := io.data_din
  data_array.GWEN := io.data_gwen_b
  data_array.WEN  := data_wen_b_all

  io.data_dout    := data_array.Q
}



class DCacheLoadTagArray extends Module {
  val io = IO(new Bundle{
    val tag_din           = Input(UInt(54.W))
    val tag_gateclk_en    = Input(Bool())
    val tag_gwen_b        = Input(Bool())
    val tag_idx           = Input(UInt(9.W))
    val tag_sel_b         = Input(Bool())
    val tag_wen_b         = Input(UInt(2.W))
    val pad_yy_icg_scan_en = Input(Bool())
    val cp0_lsu_icg_en     = Input(Bool())

    val tag_dout = Output(UInt(54.W))
  })

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val data_clk_en = io.tag_gateclk_en

  //==========================================================
  //              Instance dcache array
  //==========================================================
  val data_wen_b_all =
  Cat(VecInit(Seq.fill(27)(io.tag_wen_b(1))).asUInt, VecInit(Seq.fill(27)(io.tag_wen_b(0))).asUInt)

  val tag_array = Module(new ct_spsram_512x54)

  tag_array.A    := io.tag_idx
  tag_array.CEN  := io.tag_sel_b
  tag_array.CLK  := clock
  tag_array.D    := io.tag_din
  tag_array.GWEN := io.tag_gwen_b
  tag_array.WEN  := data_wen_b_all

  io.tag_dout    := tag_array.Q
}


class DCacheTagArray extends Module {
  val io = IO(new Bundle{
    val tag_din           = Input(UInt(52.W))
    val tag_gateclk_en    = Input(Bool())
    val tag_gwen_b        = Input(Bool())
    val tag_idx           = Input(UInt(9.W))
    val tag_sel_b         = Input(Bool())
    val tag_wen_b         = Input(UInt(2.W))
    val pad_yy_icg_scan_en = Input(Bool())
    val cp0_lsu_icg_en     = Input(Bool())

    val tag_dout = Output(UInt(52.W))
  })

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val data_clk_en = io.tag_gateclk_en

  //==========================================================
  //              Instance dcache array
  //==========================================================
  val data_wen_b_all =
  Cat(VecInit(Seq.fill(26)(io.tag_wen_b(1))).asUInt, VecInit(Seq.fill(26)(io.tag_wen_b(0))).asUInt)

  val tag_array = Module(new ct_spsram_512x52)

  tag_array.A    := io.tag_idx
  tag_array.CEN  := io.tag_sel_b
  tag_array.CLK  := clock
  tag_array.D    := io.tag_din
  tag_array.GWEN := io.tag_gwen_b
  tag_array.WEN  := data_wen_b_all

  io.tag_dout    := tag_array.Q
}

class DCacheDirtyArray extends Module {
  val io = IO(new Bundle{
    val dirty_din           = Input(UInt(7.W))
    val dirty_gateclk_en    = Input(Bool())
    val dirty_gwen_b        = Input(Bool())
    val dirty_idx           = Input(UInt(9.W))
    val dirty_sel_b         = Input(Bool())
    val dirty_wen_b         = Input(UInt(7.W))
    val pad_yy_icg_scan_en = Input(Bool())
    val cp0_lsu_icg_en     = Input(Bool())

    val dirty_dout = Output(UInt(7.W))
  })

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val data_clk_en = io.dirty_gateclk_en

  //==========================================================
  //              Instance dcache array
  //==========================================================
  val dirty_array = Module(new ct_spsram_512x52)

  dirty_array.A    := io.dirty_idx
  dirty_array.CEN  := io.dirty_sel_b
  dirty_array.CLK  := clock
  dirty_array.D    := io.dirty_din
  dirty_array.GWEN := io.dirty_gwen_b
  dirty_array.WEN  := io.dirty_wen_b

  io.dirty_dout    := dirty_array.Q
}



