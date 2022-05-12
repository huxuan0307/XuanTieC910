package Core.LSU
import chisel3._
import chisel3.util._


class IdFifo8Input extends Bundle{
  val cp0_lsu_icg_en = Bool()
  val cp0_yy_clk_en = Bool()
  val idfifo_create_id    = UInt(3.W)
  val idfifo_create_id_oh = UInt(8.W)
  val idfifo_create_vld = Bool()
  val idfifo_pop_vld = Bool()
  val pad_yy_icg_scan_en = Bool()
}

class IdFifo8Output extends Bundle{
  val idfifo_empty = Bool()
  val idfifo_pop_id_oh = UInt(8.W)
}

class IdFifo8IO extends Bundle{
  val in  = Input(new IdFifo8Input)
  val out = Output(new IdFifo8Output)
}


class IdFifo8 extends Module {
  val io = IO(new IdFifo8IO)

  //Regs
  val idfifo_create_ptr   = RegInit(0.U(4.W))
  val idfifo_pop_id_oh    = RegInit(0.U(8.W))
  val idfifo_pop_ptr      = RegInit(0.U(4.W))
  val idfifo_pop_ptr_next = RegInit(1.U(4.W))

  //Wires
  val idfifo_entry_create_vld = Wire(UInt(8.W))

  val idfifo_pe_clr_vld = Wire(Bool())
  val idfifo_pe_sel_create_ptr_vld = Wire(Bool())
  val idfifo_pop_id_next_oh = Wire(UInt(8.W))

  val idfifo_1vld = Wire(Bool())
  val idfifo_empty = Wire(Bool())
  val idfifo_create_ptr_oh = Wire(UInt(8.W))
  //==========================================================
  //                  Instance FIFO
  //==========================================================
  val idfifo_entry = RegInit(VecInit(Seq.fill(8)(0.U(3.W))))

  for(i <- 0 until 8){
    when(idfifo_entry_create_vld(i)){
      idfifo_entry(i) := io.in.idfifo_create_id
    }

  }

  //==========================================================
  //                  Register
  //==========================================================
  //------------------pointer---------------------------------
  when(io.in.idfifo_create_vld){
    idfifo_create_ptr := idfifo_create_ptr + 1.U(4.W)
  }

  when(io.in.idfifo_pop_vld){
    idfifo_pop_ptr := idfifo_pop_ptr_next
    idfifo_pop_ptr_next := idfifo_pop_ptr_next + 1.U(4.W)
  }

  when(idfifo_pe_clr_vld){
    idfifo_pop_id_oh := 0.U(8.W)
  }.elsewhen(idfifo_pe_sel_create_ptr_vld){
    idfifo_pop_id_oh := io.in.idfifo_create_id_oh
  }.elsewhen(io.in.idfifo_pop_vld){
    idfifo_pop_id_oh := idfifo_pop_id_next_oh
  }

  io.out.idfifo_pop_id_oh := idfifo_pop_id_oh
  //==========================================================
  //                    Wires
  //==========================================================
  //------------------pop entry signal------------------------
  idfifo_pe_clr_vld := io.in.idfifo_pop_vld && !io.in.idfifo_create_vld && idfifo_1vld

  idfifo_pe_sel_create_ptr_vld := io.in.idfifo_create_vld && (idfifo_empty || idfifo_1vld && io.in.idfifo_pop_vld)

  //------------------entry signal----------------------------
  idfifo_entry_create_vld := idfifo_create_ptr_oh & VecInit(Seq.fill(8)(io.in.idfifo_create_vld)).asUInt

  //------------------pointer---------------------------------
  //expand to one hot code
  idfifo_create_ptr_oh := UIntToOH(idfifo_create_ptr(2,0))(7,0)

  val idfifo_pop_ptr_next_oh = UIntToOH(idfifo_pop_ptr_next(2,0))(7,0)

  //-------------------entry valid signal---------------------
  idfifo_empty := idfifo_create_ptr === idfifo_pop_ptr
  io.out.idfifo_empty := idfifo_empty

  //only 1 entry is valid
  idfifo_1vld := idfifo_create_ptr === idfifo_pop_ptr_next

  //-------------------next pop id----------------------------
  val idfifo_pop_id_next = Mux1H(Seq(
    idfifo_pop_ptr_next_oh(0) -> idfifo_entry(0),
    idfifo_pop_ptr_next_oh(1) -> idfifo_entry(1),
    idfifo_pop_ptr_next_oh(2) -> idfifo_entry(2),
    idfifo_pop_ptr_next_oh(3) -> idfifo_entry(3),
    idfifo_pop_ptr_next_oh(4) -> idfifo_entry(4),
    idfifo_pop_ptr_next_oh(5) -> idfifo_entry(5),
    idfifo_pop_ptr_next_oh(6) -> idfifo_entry(6),
    idfifo_pop_ptr_next_oh(7) -> idfifo_entry(7)
  ))

  idfifo_pop_id_next_oh := UIntToOH(idfifo_pop_id_next(2,0))(7,0)

}
