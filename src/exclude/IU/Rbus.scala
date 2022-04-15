package Core.IU
import Core.IU.Du.DuOut
import Core.IntConfig
import chisel3._
import chisel3.util.{is, _}

class RbusIn extends Bundle {
  val aluIn = (Vec(2,new AluOut))
  val muIn = new MuOut
  val duIn = new DuOut
  val specialIn = new SpecialOut // althoug only need (data, dataVld, preg)
}
class RbusOut extends Bundle  {
  val wbPreg = UInt(7.W)
  val wbData = UInt(XLEN.W)
  val wbPregVld = Bool()
}

class RbusIO extends Bundle {
  val in = Input(new RbusIn)
  val flush = Input(Bool())
  val out = Output(Vec(3, new RbusOut))
}
class Rbus extends Module with IUConfig {
  val io = IO(new RbusIO)
  val pipe_rslt_vld = Seq.fill(IuPipeNum)(Wire(Bool()))
  pipe_rslt_vld.head := io.in.aluIn(0).dataVld || io.in.duIn.pipe0DataVld || io.in.specialIn.dataVld
  pipe_rslt_vld(1)   := io.in.aluIn(1).dataVld || io.in.muIn.dataVld
  //----------------------------------------------------------
  //                   Write Back Valid
  //----------------------------------------------------------
  val pipe_wb_vld = Seq.fill(IuPipeNum)(RegInit(false.B))
  for(i<- 0 until IuPipeNum){
    when(io.flush){
      pipe_wb_vld(i) := false.B
    }otherwise{
      pipe_wb_vld(i) := pipe_rslt_vld(i)
    }
  }
  //----------------------------------------------------------
  //                Write Back Data Selection
  //----------------------------------------------------------
  //----------------------------------------------------------
  //                         Pipe 0
  //----------------------------------------------------------
  val pipe0_select = Cat(io.in.aluIn(0).dataVld ,io.in.duIn.pipe0DataVld ,io.in.specialIn.dataVld) // TODO add CP0
  val pipe0_rslt_preg = RegInit(0.U(7.W))
  switch(pipe0_select){
    is("b100".U){
      pipe0_rslt_preg := io.in.aluIn(0).preg
  }
    is("b010".U){
      pipe0_rslt_preg := io.in.duIn.preg
    }
    is("b001".U){
      pipe0_rslt_preg := io.in.specialIn.preg
    }
  }
  val pipe0_rslt_data = RegInit(0.U(XLEN.W))
  switch(pipe0_select){
    is("b100".U){
      pipe0_rslt_data := io.in.aluIn(0).data
    }
    is("b010".U){
      pipe0_rslt_data := io.in.duIn.data
    }
    is("b001".U){
      pipe0_rslt_data := io.in.specialIn.data
    }
  }
  val pipe0_wb_rslt_preg = RegEnable(pipe0_rslt_preg, pipe_rslt_vld.head)
  val pipe0_wb_rslt_data = RegEnable(pipe0_rslt_data, pipe_rslt_vld.head)
  io.out(0).wbPreg    := pipe0_wb_rslt_preg
  io.out(0).wbData    := pipe0_wb_rslt_data
  io.out(0).wbPregVld := pipe_wb_vld(0)
  //----------------------------------------------------------
  //                         Pipe 1
  //----------------------------------------------------------
  val pipe1_select = Cat(io.in.aluIn(1).dataVld ,io.in.muIn.dataVld) // TODO add CP0
  val pipe1_rslt_preg = RegInit(0.U(7.W))
  switch(pipe1_select){
    is("b10".U){
      pipe1_rslt_preg := io.in.aluIn(1).preg
    }
    is("b01".U){
      pipe1_rslt_preg := io.in.muIn.preg
    }
  }
  val pipe1_rslt_data = RegInit(0.U(XLEN.W))
  switch(pipe1_select){
    is("b10".U){
      pipe1_rslt_data := io.in.aluIn(1).data
    }
    is("b01".U){
      pipe1_rslt_data := io.in.muIn.data
    }
  }
  val pipe1_wb_rslt_preg = RegEnable(pipe1_rslt_preg, pipe_rslt_vld(1))
  val pipe1_wb_rslt_data = RegEnable(pipe1_rslt_data, pipe_rslt_vld(1))
  io.out(1).wbPreg    := pipe1_wb_rslt_preg
  io.out(1).wbData    := pipe1_wb_rslt_data
  io.out(1).wbPregVld := pipe_wb_vld(1)

}
