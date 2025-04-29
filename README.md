## Single-Cycle RISC-V Processor with AXI Stream Interface

This project implements a **single-cycle RISC-V processor** integrated with an **AXI Stream interface** to enable external communication via streaming data. The system is designed using Verilog HDL and focuses on combining a basic CPU architecture with a hardware-based output mechanism for external data transmission.

### Objective

The main objective is to design and simulate a fully functional RISC-V processor that:
- Executes a minimal RISC-V instruction set in a single clock cycle per instruction.
- Detects memory-mapped store operations to a specific AXI address range.
- Converts 32-bit data into 8-bit streamable chunks.
- Interfaces with AXI Stream Master and Slave logic for output and feedback verification.

---

### System Components

The system consists of the following key components:

#### 1. **Single-Cycle RISC-V CPU Core**
- Implements core stages (Fetch, Decode, Execute, Memory, Writeback) in a single cycle.
- Supports instructions: `LUI`, `ADDI`, `SW`, `JAL`, `NOP`, etc.
- Includes modules:
  - **Instruction Memory** and **Program Counter**
  - **Control Unit** and **ALU Control**
  - **Register File** and **ALU**
  - **Immediate Generator**
  - **Data Memory** with address decoding

#### 2. **Memory-Mapped AXI Trigger**
- An **Address Decoder** detects `SW` instructions targeting `0xFFFFF800`.
- This triggers a `newd` signal to indicate AXI transfer initiation.

#### 3. **32-to-8-bit FIFO Buffer**
- Splits 32-bit data written to the AXI region into four 8-bit segments.
- Allows clocked, sequential output to match AXI streaming protocol.

#### 4. **AXI Master Interface (`axis_m`)**
- Transmits 8-bit data over AXI when `newd` is high.
- Implements basic state machine (idle/tx) with byte counting and end-of-transfer signaling.

#### 5. **AXI Slave Interface (`axis_s`)**
- Receives the streamed data (loopback simulation).
- Acknowledges data readiness and indicates when transmission is complete.

---

### Operation Flow

1. The RISC-V core executes instructions from memory.
2. When a store (`sw`) instruction targets `0xFFFFF800`, the address decoder asserts `newd`.
3. Data intended for output is written to the FIFO.
4. The AXI Master reads data from FIFO and sends it as a sequence of 8-bit transactions.
5. The AXI Slave acknowledges receipt (simulated in loopback configuration).
6. The process repeats as long as the program loops (via `jal`).

---

### Simulation Results

- Simulation was performed using a testbench (`toptwo_tb`) that verifies:
  - Proper instruction execution
  - ALU results
  - FIFO loading and unloading
  - AXI transaction signals (`tvalid`, `tdata`, `tlast`, `tready`)
- `sim results.txt` includes timestamped signal snapshots.
- Waveform confirms timing behavior of the processor and streaming interface.

![image](https://github.com/user-attachments/assets/b4b58cfe-de23-4378-a154-7326dcf160d4)

### RTL Diagram on intel quartus prime
![image](https://github.com/user-attachments/assets/a59aa9d8-7ea9-4a1f-9d72-ba98363488cd)



