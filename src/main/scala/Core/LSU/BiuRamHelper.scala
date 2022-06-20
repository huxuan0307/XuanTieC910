package Core.LSU

import Core.Config.PcStart
import Core.LsuConfig
import chisel3._
import chisel3.util._

class biu_r extends Bundle{
  val data = UInt(128.W)
  val id   = UInt(5.W)
  val last = Bool()
  val resp = UInt(4.W)
  val vld  = Bool()
}

class biu_b extends Bundle{
  val id = UInt(5.W)
  val resp = UInt(2.W)
  val vld = Bool()
}

class RAMHelper extends BlackBox {
  val io = IO(new Bundle {
    val clk = Input(Clock())
    val en = Input(Bool())

    val rIdx = Input(UInt(64.W))
    val rdata = Output(UInt(64.W))

    val wIdx = Input(UInt(64.W))
    val wdata = Input(UInt(64.W))
    val wmask = Input(UInt(64.W))
    val wen = Input(Bool())
  })
}

class BiuRamHelperInput extends Bundle{
  val ar = new biu_ar

  val st_aw = new biu_aw
  val vict_aw = new biu_aw

  val st_w = new biu_w
  val vict_w = new biu_w
}

class BiuRamHelperOutput extends Bundle{
  val BiuGrnt = new Bundle{
    val ar_ready = Bool()
    val aw_vb_grnt = Bool()
    val aw_wmb_grnt = Bool()
    val w_vb_grnt = Bool()
    val w_wmb_grnt = Bool()
  }

  val r = new biu_r

  val b = new biu_b
}

class BiuRamHelperIO extends Bundle{
  val in  = Input(new BiuRamHelperInput)
  val out = Output(new BiuRamHelperOutput)
}

class BiuRamHelper extends Module with LsuConfig{
  val io = IO(new BiuRamHelperIO)

  //state machine
  val r_idle :: r_resp :: Nil = Enum(2)
  val w_idle :: w_data :: w_resp :: Nil = Enum(3)

  //read
  val read_state = RegInit(r_idle)
  val read_info  = RegInit(0.U.asTypeOf(new biu_ar))
  val read_cnt   = RegInit(0.U(4.W))
  val read_addr_to6  = RegInit(0.U((PA_WIDTH-6).W))
  val read_addr_5to4 = RegInit(0.U(2.W))

  when(io.in.ar.req && read_state === r_idle){
    read_state := r_resp
    read_info  := io.in.ar
    read_cnt   := io.in.ar.len + 1.U(4.W)
    read_addr_to6  := (io.in.ar.addr - PcStart.U)(PA_WIDTH-1,6)
    read_addr_5to4 := io.in.ar.addr(5,4)
  }

  when(read_state === r_resp){
    read_cnt  := read_cnt - 1.U
    read_addr_5to4 := read_addr_5to4 + 1.U

    when(read_cnt === 1.U){
      read_state := r_idle
    }
  }

  val read_addr = Cat(read_addr_to6, read_addr_5to4, 0.U(4.W))
  val read_ramhelper = Seq.fill(2)(Module(new RAMHelper))
  val read_data = Wire(Vec(2, UInt(64.W)))
  for(i <- 0 until 2){
    read_ramhelper(i).io.clk  := clock
    read_ramhelper(i).io.en   := !reset.asBool
    read_ramhelper(i).io.rIdx := Mux(i.U === 0.U, read_addr >> 3.U, (read_addr + 8.U) >> 3.U)
    read_ramhelper(i).io.wIdx  := 0.U
    read_ramhelper(i).io.wdata := 0.U
    read_ramhelper(i).io.wmask := 0.U
    read_ramhelper(i).io.wen   := false.B

    read_data(i) := read_ramhelper(i).io.rdata
  }
  io.out.BiuGrnt.ar_ready := io.in.ar.req && read_state === r_idle

  io.out.r.data := read_data.asUInt
  io.out.r.id   := read_info.id
  io.out.r.last := read_state === r_resp && read_cnt === 1.U
  io.out.r.resp := 0.U
  io.out.r.vld  := read_state === r_resp

  //write VB > WMB
  //VB
  val vb_state = RegInit(w_idle)
  val vb_info  = RegInit(0.U.asTypeOf(new biu_aw))
  val vb_cnt   = RegInit(0.U(4.W))
  val vb_addr_to6  = RegInit(0.U((PA_WIDTH-6).W))
  val vb_addr_5to4 = RegInit(0.U(2.W))

  when(io.in.vict_aw.req && vb_state === w_idle){
    vb_state := w_data
    vb_info  := io.in.vict_aw
    vb_cnt   := io.in.vict_aw.len + 1.U(4.W)
    vb_addr_to6  := (io.in.vict_aw.addr - PcStart.U)(PA_WIDTH-1,6)
    vb_addr_5to4 := io.in.vict_aw.addr(5,4)
  }

  when(io.in.vict_w.vld && vb_state === w_data){
    vb_cnt  := vb_cnt - 1.U
    vb_addr_5to4 := vb_addr_5to4 + 1.U

    when(io.in.vict_w.last && vb_cnt === 1.U){
      vb_state := w_resp
    }
  }

  val vb_addr = Cat(vb_addr_to6, vb_addr_5to4, 0.U(4.W))
  val vb_ramhelper = Seq.fill(2)(Module(new RAMHelper))
  val vb_data = io.in.vict_w.data.asTypeOf(Vec(2, UInt(64.W)))
  val vb_strb = Wire(Vec(16, UInt(8.W)))
  for(i <- 0 until 16){
    vb_strb(i) := VecInit(Seq.fill(8)(io.in.vict_w.strb(i))).asUInt
  }
  for(i <- 0 until 2){
    vb_ramhelper(i).io.clk  := clock
    vb_ramhelper(i).io.en   := !reset.asBool
    vb_ramhelper(i).io.rIdx := 0.U
    vb_ramhelper(i).io.wIdx  := Mux(i.U === 0.U, vb_addr >> 3.U, (vb_addr + 8.U) >> 3.U)
    vb_ramhelper(i).io.wdata := vb_data(i)
    vb_ramhelper(i).io.wmask := vb_strb.asUInt(i*64+63, i*64)
    vb_ramhelper(i).io.wen   := io.in.vict_w.vld && vb_state === w_data
  }

  io.out.BiuGrnt.aw_vb_grnt := io.in.vict_aw.req && vb_state === w_idle
  io.out.BiuGrnt.w_vb_grnt  := io.in.vict_w.vld && vb_state === w_data


  //WMB
  val wmb_state = RegInit(w_idle)
  val wmb_info  = RegInit(0.U.asTypeOf(new biu_aw))
  val wmb_cnt   = RegInit(0.U(4.W))
  val wmb_addr_to6  = RegInit(0.U((PA_WIDTH-6).W))
  val wmb_addr_5to4 = RegInit(0.U(2.W))

  when(io.in.st_aw.req && wmb_state === w_idle){
    wmb_state := w_data
    wmb_info  := io.in.st_aw
    wmb_cnt   := io.in.st_aw.len + 1.U(4.W)
    wmb_addr_to6  := (io.in.st_aw.addr - PcStart.U)(PA_WIDTH-1,6)
    wmb_addr_5to4 := io.in.st_aw.addr(5,4)
  }

  when(io.in.st_w.vld && wmb_state === w_data){
    wmb_cnt  := wmb_cnt - 1.U
    wmb_addr_5to4 := wmb_addr_5to4 + 1.U

    when(io.in.st_w.last && wmb_cnt === 1.U){
      wmb_state := w_resp
    }
  }

  val wmb_addr = Cat(wmb_addr_to6, wmb_addr_5to4, 0.U(4.W))
  val wmb_ramhelper = Seq.fill(2)(Module(new RAMHelper))
  val wmb_data = io.in.st_w.data.asTypeOf(Vec(2, UInt(64.W)))
  val wmb_strb = Wire(Vec(16, UInt(8.W)))
  for(i <- 0 until 16){
    wmb_strb(i) := VecInit(Seq.fill(8)(io.in.st_w.strb(i))).asUInt
  }
  for(i <- 0 until 2){
    wmb_ramhelper(i).io.clk  := clock
    wmb_ramhelper(i).io.en   := !reset.asBool
    wmb_ramhelper(i).io.rIdx := 0.U
    wmb_ramhelper(i).io.wIdx  := Mux(i.U === 0.U, wmb_addr >> 3.U, (wmb_addr + 8.U) >> 3.U)
    wmb_ramhelper(i).io.wdata := wmb_data(i)
    wmb_ramhelper(i).io.wmask := wmb_strb.asUInt(i*64+63, i*64)
    wmb_ramhelper(i).io.wen   := io.in.st_w.vld && wmb_state === w_data
  }

  io.out.BiuGrnt.aw_wmb_grnt := io.in.st_aw.req && wmb_state === w_idle
  io.out.BiuGrnt.w_wmb_grnt  := io.in.st_w.vld && wmb_state === w_data


  //write resp
  io.out.b.id   := Mux(vb_state === w_resp, vb_info.id, wmb_info.id)
  io.out.b.resp := 0.U
  io.out.b.vld  := vb_state === w_resp || wmb_state === w_resp

  when(vb_state === w_resp){
    vb_state := w_idle
  }.elsewhen(wmb_state === w_resp){
    wmb_state := w_idle
  }

}
