package Core.IFU
import Utils._
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class IbufPtr extends CircularQueuePtr[IbufPtr](Config.IBufSize)

class IBufferData extends CoreBundle {
  val pc   = UInt(VAddrBits.W)
  val data = UInt(16.W)
  val is_inst32 = Bool()
}

class IBufferData_data extends CoreBundle {
  val pc   = UInt(VAddrBits.W)
  val data = UInt(16.W)
  val is_inst32 = Bool()
}

class IBuf2Decode extends CoreBundle {
  val pc   = UInt(VAddrBits.W)
  val inst = UInt(32.W)
}

class IBufferIO extends CoreBundle {
  val in   = Vec(8+1, Flipped(Valid(new IBufferData)))
  val out  = Vec(3, Valid(new IBuf2Decode))
  val allowEnq = Output(Bool())
  val flush = Input(Bool())
}

class IBuffer extends Module with Config with HasCircularQueuePtrHelper {
  val io = IO(new IBufferIO)

  val valid = RegInit(VecInit(Seq.fill(IBufSize)(false.B)))
  //  val data  = RegInit(VecInit(Seq.fill(IBufSize)(0.U.asTypeOf(new IBufferData_data))))
  //  val valid = Mem(IBufSize,Bool())
  //  val data = Mem(IBufSize,UInt(16.W))
  val data = RegInit(VecInit(Seq.fill(IBufSize)(0.U(16.W))))
  //  val pc = Mem(IBufSize,UInt(VAddrBits.W))
  val pc = RegInit(VecInit(Seq.fill(IBufSize)(0.U(VAddrBits.W))))
  val is_inst32 = RegInit(VecInit(Seq.fill(IBufSize)(false.B)))
//  val is_inst32 = Mem(IBufSize,Bool())
  val enqPtr = RegInit(0.U.asTypeOf(new IbufPtr))
  val deqPtr = RegInit(0.U.asTypeOf(new IbufPtr))

  val validEntries = distanceBetween(enqPtr, deqPtr)

  //Enq
  val enq_num = PopCount(io.in.map(_.valid))
  io.allowEnq := validEntries <= IBufSize.U - 9.U

  when(io.in(0).valid && io.allowEnq) {
    for (i <- 0 until 8 + 1) {
      valid(enqPtr.value + i.U) := io.in(i).valid
//      data.write(enqPtr.value + i.U,io.in(i).bits.data)
      data(enqPtr.value + i.U) := io.in(i).bits.data
//      pc.write(enqPtr.value + i.U,io.in(i).bits.pc)
      pc(enqPtr.value + i.U) := io.in(i).bits.pc
//      is_inst32.write(enqPtr.value + i.U,io.in(i).bits.is_inst32)
      is_inst32(enqPtr.value + i.U) := io.in(i).bits.is_inst32
//      data(enqPtr.value + i.U).data := io.in(i).bits.data
//      data(enqPtr.value + i.U).pc := io.in(i).bits.pc
//      data(enqPtr.value + i.U).is_inst32 := io.in(i).bits.is_inst32
//      data(enqPtr.value + i.U).ena := true.B
    }
  }.elsewhen(io.in(1).valid && !io.in(0).valid && io.allowEnq) {
    for (i <- 0 until 8) {
//      data.write(enqPtr.value + i.U,io.in(i+1).bits.data)
      data(enqPtr.value + i.U) := io.in(i+1).bits.data
//      pc.write(enqPtr.value + i.U,io.in(i+1).bits.pc)
      pc(enqPtr.value + i.U) := io.in(i+1).bits.pc
//      is_inst32.write(enqPtr.value + i.U,io.in(i+1).bits.is_inst32)
      is_inst32(enqPtr.value + i.U) := io.in(i+1).bits.is_inst32
      //      valid.write(enqPtr.value + i.U,io.in(i + 1).valid)
      valid(enqPtr.value + i.U) := io.in(i + 1).valid
//      data(enqPtr.value + i.U).data := io.in(i+1).bits.data
//      data(enqPtr.value + i.U).pc := io.in(i+1).bits.pc
//      data(enqPtr.value + i.U).is_inst32 := io.in(i+1).bits.is_inst32
//      data(enqPtr.value + i.U).ena := true.B
    }
  }

  when(io.allowEnq){
    enqPtr := enqPtr + enq_num
  }

  //Deq
//  val deq_vec = WireInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new IbufPtr))))
//  deq_vec(0) := deqPtr
//  deq_vec(1) := deqPtr + valid(deq_vec(0).value) + (valid(deq_vec(0).value) && is_inst32(deq_vec(0).value))//data(deq_vec(0).value).is_inst32)
//  deq_vec(2) := deq_vec(1) + valid(deq_vec(1).value) + (valid(deq_vec(1).value) && is_inst32(deq_vec(1).value))//data(deq_vec(1).value).is_inst32)
  val deq_vec = WireInit(VecInit(Seq.fill(3)(0.U(5.W))))
  deq_vec(0) := deqPtr.value
  deq_vec(1) := deqPtr.value + valid(deqPtr.value).asUInt() + (valid(deqPtr.value) && is_inst32(deqPtr.value)).asUInt()
  deq_vec(2) := deq_vec(1) + valid(deq_vec(1)).asUInt() + (valid(deq_vec(1)) && is_inst32(deq_vec(1))).asUInt()
//  deq_vec(0).value := 0.U
//  deq_vec(1).value := 2.U
//  deq_vec(2).value := 4.U
  val deq_ena = VecInit(Seq.fill(3)(0.U))
  for(i <- 0 until 3){
//    when(data(deq_vec(i).value).ena) {
//      io.out(i).valid     := valid(deq_vec(i).value)
//      io.out(i).bits.pc   := data(deq_vec(i).value).pc
//      io.out(i).bits.inst := Cat(data(deq_vec(i).value+1.U).data,data(deq_vec(i).value).data)
//      data(deq_vec(i).value).ena := false.B
//    }.otherwise {
//      io.out(i).valid     := false.B
//      io.out(i).bits.pc   := 0.U
//      io.out(i).bits.inst := 0.U
//      data(deq_vec(i).value).ena := false.B
//    }
    io.out(i).valid     := valid(deq_vec(i))
    io.out(i).bits.pc   := pc(deq_vec(i))//data(deq_vec(i).value).pc
    io.out(i).bits.inst := Cat(data(deq_vec(i)+1.U),data(deq_vec(i)))//Cat(data(deq_vec(i).value+1.U).data,data(deq_vec(i).value).data)
//    io.out(i).bits.inst := Cat(Mux(data(deq_vec(i).value).is_inst32, data(deq_vec(i).value+1.U).data, 0.U(16.W)), data(deq_vec(i).value).data)
  }

  val deq_num = WireInit(VecInit(Seq.fill(6)(false.B)))
  for(i <- 0 until 3){
    deq_num(2*i)   := io.out(i).fire
//    deq_num(2*i+1) := io.out(i).fire && is_inst32.read(deq_vec(i).value)//data(deq_vec(i).value).is_inst32
    deq_num(2*i+1) := io.out(i).fire && is_inst32(deq_vec(i))
  }
  deqPtr := deqPtr + PopCount(deq_num)

  //flush
  when(io.flush){
    enqPtr := 0.U.asTypeOf(new IbufPtr)
    deqPtr := 0.U.asTypeOf(new IbufPtr)
//    valid  := VecInit(Seq.fill(IBufSize)(false.B))
  }

}
