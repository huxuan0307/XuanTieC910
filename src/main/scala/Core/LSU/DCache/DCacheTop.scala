package Core.LSU.DCache
import chisel3._
import chisel3.util._

class DCacheTopInput extends Bundle{
  val cp0_lsu_icg_en = Bool()
  val pad_yy_icg_scan_en = Bool()

  val ld_data = new Bundle{
    val gateclk_en = Vec(8, Bool())
    val gwen_b = Vec(8, Bool())
    val high_din = UInt(128.W)
    val high_idx = UInt(11.W)
    val low_din = UInt(128.W)
    val low_idx = UInt(11.W)
    val sel_b = Vec(8, Bool())
    val wen_b = Vec(8, UInt(4.W))
  }
  val ld_tag = new Bundle{
    val din = UInt(54.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(2.W)
  }
  val st_dirty = new Bundle{
    val din = UInt(7.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(7.W)
  }
  val st_tag = new Bundle{
    val din = UInt(52.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(2.W)
  }
}

class DCacheTopOutput extends Bundle{
  val ld_data_bank_dout = Vec(8, UInt(32.W))
  val ld_tag_dout = UInt(54.W)
  val st_dirty_dout = UInt(7.W)
  val st_tag_dout = UInt(52.W)
}

class DCacheIO extends Bundle{
  val in = Input(new DCacheTopInput)
  val out = Output(new DCacheTopOutput)
}

class DCacheTop extends Module {
  val io = IO(new DCacheIO)

  //==========================================================
  //                Instance dcache array
  //==========================================================
  //---------------------tag and dirty------------------------
  val ld_tag_array = Module(new DCacheLoadTagArray)
  ld_tag_array.io.tag_din            := io.in.ld_tag.din
  ld_tag_array.io.tag_gateclk_en     := io.in.ld_tag.gateclk_en
  ld_tag_array.io.tag_gwen_b         := io.in.ld_tag.gwen_b
  ld_tag_array.io.tag_idx            := io.in.ld_tag.idx
  ld_tag_array.io.tag_sel_b          := io.in.ld_tag.sel_b
  ld_tag_array.io.tag_wen_b          := io.in.ld_tag.wen_b
  ld_tag_array.io.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
  ld_tag_array.io.cp0_lsu_icg_en     := io.in.cp0_lsu_icg_en
  io.out.ld_tag_dout := ld_tag_array.io.tag_dout


  val st_tag_array = Module(new DCacheTagArray)
  st_tag_array.io.tag_din            := io.in.st_tag.din
  st_tag_array.io.tag_gateclk_en     := io.in.st_tag.gateclk_en
  st_tag_array.io.tag_gwen_b         := io.in.st_tag.gwen_b
  st_tag_array.io.tag_idx            := io.in.st_tag.idx
  st_tag_array.io.tag_sel_b          := io.in.st_tag.sel_b
  st_tag_array.io.tag_wen_b          := io.in.st_tag.wen_b
  st_tag_array.io.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
  st_tag_array.io.cp0_lsu_icg_en     := io.in.cp0_lsu_icg_en
  io.out.st_tag_dout := st_tag_array.io.tag_dout


  val st_dirty_array = Module(new DCacheDirtyArray)
  st_dirty_array.io.dirty_din            := io.in.st_dirty.din
  st_dirty_array.io.dirty_gateclk_en     := io.in.st_dirty.gateclk_en
  st_dirty_array.io.dirty_gwen_b         := io.in.st_dirty.gwen_b
  st_dirty_array.io.dirty_idx            := io.in.st_dirty.idx
  st_dirty_array.io.dirty_sel_b          := io.in.st_dirty.sel_b
  st_dirty_array.io.dirty_wen_b          := io.in.st_dirty.wen_b
  st_dirty_array.io.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
  st_dirty_array.io.cp0_lsu_icg_en     := io.in.cp0_lsu_icg_en
  io.out.st_dirty_dout := st_dirty_array.io.dirty_dout


  //-------------------------data-----------------------------
  val ld_data_bank_array = Seq.fill(8)(Module(new DCacheDataArray))

  for(i <- 0 until 4){
    ld_data_bank_array(i).io.data_din   := io.in.ld_data.low_din(32*i+31, 32*i)
    ld_data_bank_array(i).io.data_idx   := io.in.ld_data.low_idx
    ld_data_bank_array(i+4).io.data_din := io.in.ld_data.high_din(32*i+31, 32*i)
    ld_data_bank_array(i+4).io.data_idx := io.in.ld_data.high_idx
  }

  for(i <- 0 until 8){
    ld_data_bank_array(i).io.data_gateclk_en := io.in.ld_data.gateclk_en(i)
    ld_data_bank_array(i).io.data_gwen_b := io.in.ld_data.gwen_b(i)
    ld_data_bank_array(i).io.data_sel_b := io.in.ld_data.sel_b(i)
    ld_data_bank_array(i).io.data_wen_b := io.in.ld_data.wen_b(i)
    ld_data_bank_array(i).io.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
    ld_data_bank_array(i).io.cp0_lsu_icg_en := io.in.cp0_lsu_icg_en

    io.out.ld_data_bank_dout(i) := ld_data_bank_array(i).io.data_dout
  }

}
