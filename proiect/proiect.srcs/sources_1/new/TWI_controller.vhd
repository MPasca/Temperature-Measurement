----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/28/2021 03:05:39 PM
-- Design Name: 
-- Module Name: TWI_controller - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

-- newly defined package
package TWIUtils is
  type busState_type is (busUnknown, busBusy, busFree);
  type error_type is (errArb, errNAck);
end TWIUtils;

package body TWIUtils is 
end TWIUtils;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.math_real.all;

-- using the newly defined package
use work.TWIUtils.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity TWI_controller is
	Generic (
        CLKFREQ : natural := 50;
        ATTEMPT_SLAVE_UNBLOCK : boolean := false);
    
    Port (clk : in std_logic;
          rst : in std_logic;
    
            MSG_I : in STD_LOGIC; -- new message
            STB_I : in STD_LOGIC; -- strobe
            
            A_I : in  STD_LOGIC_VECTOR (7 downto 0);    -- address input bus
            D_I : in  STD_LOGIC_VECTOR (7 downto 0);    -- data input bus
            D_O : out  STD_LOGIC_VECTOR (7 downto 0);   -- data output bus
            
            done_signal : out  STD_LOGIC;   -- done status signal
            error_signal : out  STD_LOGIC;  -- error status
            errortype : out error_type;     -- error type

        -- bus signals
            SDA : inout std_logic;  -- TWI SDA
            SCL : inout std_logic   -- TWI SCL
        );
end TWI_controller;

architecture Behavioral of TWI_controller is
    attribute fsm_encoding: string;
    
    constant FSCL : natural := 400_000;     -- in Hz SCL clock frequency
        constant TIMEOUT : natural := 10;   -- in ms TWI timeout for slave wait period
        constant TSCL_CYCLES : natural := 
            natural(ceil(real(CLKFREQ*1_000_000/FSCL)));
        constant TIMEOUT_CYCLES : natural :=
            natural(ceil(real(CLKFREQ*TIMEOUT*1_000)));
            
    type state_type is (idle, start, read, write, error, stop,
                    SAck, MAck, MNAckStop, MNAckStart, stopErr); 
                    
    signal state, nextState : state_type;
    attribute fsm_encoding of state: signal is "gray";    
        
    signal sync_sda, sync_scl : STD_LOGIC_VECTOR(2 downto 0);
    signal dSDA, ddSDA, dSCL : STD_LOGIC;
    signal fStart, fStop : STD_LOGIC;
    signal busState : busState_type := busUnknown;
    signal errTypeR, errType : error_type;
    signal busFreeCnt, sclCnt : natural range TSCL_CYCLES downto 0 := TSCL_CYCLES;
    signal timeOutCnt : natural range TIMEOUT_CYCLES downto 0 := TIMEOUT_CYCLES;
    
    signal slaveWait, arbLost : STD_LOGIC;
    signal dataByte, loadByte, currAddr : STD_LOGIC_VECTOR(7 downto 0); -- shift register and parallel load
    signal rSDA, rSCL : STD_LOGIC := '1';
    signal subState : STD_LOGIC_VECTOR(1 downto 0) := "00";
    signal latchData, latchAddr, iDone, iErr, iSda, iScl, shiftBit, dataBitOut, rwBit, addrNData : STD_LOGIC;
    signal bitCount : natural range 0 to 7 := 7;
    signal int_Rst : STD_LOGIC := '0';
begin

-- bus state
SYNC_FFS: process(CLK)
   begin
      if Rising_Edge(CLK) then
			sync_sda(0) <= SDA;
			sync_sda(1) <= sync_sda(0);
			sync_sda(2) <= sync_sda(1);
			sync_scl(0) <= SCL;
			sync_scl(1) <= sync_scl(0);
			sync_scl(2) <= sync_scl(1);
      end if;
end process;
	
	dSDA <= sync_sda(1);
	ddSDA <= sync_sda(2);
	dSCL <= sync_scl(1);
	
	fStart <= dSCL and not dSDA and ddSDA; -- SCL high & SDA falling => start
	fStop <= dSCL and dSDA and not ddSDA; -- SCL high & SDA rising => stop

TWI_STATE: process(CLK)
   begin
      if Rising_Edge(CLK) then
			if int_Rst = '1' then
				busState <= busUnknown;
         elsif fStart = '1' then -- START condition detected => bus is busy
            busState <= busBusy;
        elsif busFreeCnt = 0 then -- tBUF => free
            busState <= busFree;
         end if;
      end if;
end process;

TBUF_CNT: process(CLK)
   begin
      if Rising_Edge(CLK) then
         if dSCL = '0' or dSDA = '0' or int_Rst = '1' then
            busFreeCnt <= TSCL_CYCLES;
         elsif dSCL = '1' and dSDA = '1' then
            busFreeCnt <= busFreeCnt - 1;
         end if;
      end if;
end process;
   
slaveWait <= '1' when (dSCL = '0' and rSCL = '1') else '0';
arbLost <=   '1' when (dSCL = '1' and dSDA = '0' and rSDA = '1') else '0';

-- reset
RST_PROC: process (clk)
   begin
      if Rising_Edge(clk) then
         if state = idle and RST = '0' then
            int_Rst <= '0';
         elsif RST = '1' then
            int_Rst <= '1';
         end if;
      end if;
end process;
   
-- counter SCL
SCL_CNT: process (clk)
	begin
		if Rising_Edge(clk) then
			if sclCnt = 0 or state = idle then
				sclCnt <= TSCL_CYCLES/4;
			elsif slaveWait = '0' then -- clk sync with masters
				sclCnt <= sclCnt - 1;
			end if;
		end if;
end process;
	
UnblockTimeout: if ATTEMPT_SLAVE_UNBLOCK generate
    TIMEOUT_CNT: process (CLK)
        begin
            if Rising_Edge(CLK) then
                if state /= idle or busState = busFree or ((ddSDA xor dSDA) = '1') then
                    timeOutCnt <= TIMEOUT_CYCLES;
                else
                    timeOutCnt <= timeOutCnt - 1;
                end if;
            end if;
    end process;
end generate;
    
DATABYTE_SHREG: process (clk) 
        begin
            if Rising_Edge(clk) then
                if (latchData = '1' or latchAddr = '1') and sclCnt = 0 then
                    dataByte <= loadByte; --latch address/data
                    bitCount <= 7;
                    -- flags to recognize the sent byte
                    if latchData = '1' then
                        addrNData <= '0';
                    else
                        addrNData <= '1';
                    end if;
                elsif shiftBit = '1' and sclCnt = 0 then
                    dataByte <= dataByte(dataByte'high-1 downto 0) & dSDA;
                    bitCount <= bitCount - 1;
                end if;
            end if;
end process;
    
        loadByte <= A_I when latchAddr = '1' else
                        D_I;
        dataBitOut <= dataByte(dataByte'high);
        
        D_O <= dataByte;
        
CURRADDR_REG: process (CLK) 
            begin
                if Rising_Edge(CLK) then
                    if latchAddr = '1' then
                        currAddr <= A_I; --latch address/data
                    end if;
                end if;
end process;
            
    rwBit <= currAddr(0);
    
SUBSTATE_CNT: process (CLK)
           begin
              if Rising_Edge(CLK) then
                    if state = idle then
                        subState <= "00";
                    elsif sclCnt = 0 then
                        subState <= subState + 1;
                    end if;
                end if;
            end process;
            
        SYNC_PROC: process (CLK)
           begin
              if Rising_Edge(CLK) then
                 state <= nextState;
                    
                    rSda <= iSda;
                 rScl <= iScl;
                    if int_Rst = '1' then
                        done_signal <= '0';
                        error_signal <= '0';
                        errorType <= errType;
                    else
                        done_signal <= iDone;
                        error_signal <= iErr;
                        errTypeR <= errType;
                    end if;
              end if;
end process;      

OUTPUT_DECODE: process(nextState, subState, state, errTypeR, dataByte(0), sclCnt, bitCount, rSDA, rSCL, dataBitOut, arbLost, dSDA, addrNData)
   begin
		iSDA <= rSDA;
		iSCL <= rSCL;
		iDone <= '0';
		iErr <= '0';
		errType <= errTypeR;
		shiftBit <= '0';
		latchAddr <= '0';
		latchData <= '0';
		
		if state = start then
			case subState is
				when "00" =>
					iSDA <= '1';
				when "01" =>
					iSDA <= '1';
					iSCL <= '1';
				when "10" =>
					iSDA <= '0';
					iSCL <= '1';
				when "11" =>
					iSDA <= '0';
					iSCL <= '0';
				when others =>
			end case;
		end if;
		
		if state = stop or state = stopErr then
			case subState is
				when "00" =>
					iSDA <= '0';
				when "01" =>
					iSDA <= '0';
					iSCL <= '1';
				when "10" =>
					iSDA <= '1';
					iSCL <= '1';
				when "11" => -- arbitration error
					iSCL <= '0'; -- toggle clock
				when others =>					
			end case;
		end if;
		
		if state = read or state = SAck then
			case subState is
				when "00" =>
					iSDA <= '1'; --this will be 'Z' on SDA
					--keep SCL
				when "01" =>
					--keep SDA
					iSCL <= '1';
				when "10" =>
					--keep SDA
					iSCL <= '1';
				when "11" =>
					--keep SDA
					iSCL <= '0';
				when others =>					
			end case;
		end if;
		
		if state = write then
			case subState is
				when "00" =>
					iSDA <= dataBitOut;
				when "01" =>
					iSCL <= '1';
				when "10" =>
					iSCL <= '1';
				when "11" =>
					iSCL <= '0';
				when others =>					
			end case;
		end if;
		
		if state = MAck then
			case subState is
				when "00" =>
					iSDA <= '0'; 
				when "01" =>
					iSCL <= '1';
				when "10" =>
					iSCL <= '1';
				when "11" =>
					iSCL <= '0';
				when others =>					
			end case;
		end if;
		
		if state = MNAckStop or state = MNAckStart then
			case (subState) is
				when "00" =>
					iSDA <= '1';
				when "01" =>
					iSCL <= '1';
				when "10" =>
					iSCL <= '1';
				when "11" =>
					iSCL <= '0';
				when others =>					
			end case;
		end if;
		
		if state = SAck and sclCnt = 0 and subState = "01" then
			if dSDA = '1' then
				iDone <= '1';
				iErr <= '1';
				errType <= errNAck;
			elsif addrNData = '0' then
				iDone <= '1';
			end if;
		end if;
		
		if state = read and subState = "01" and sclCnt = 0 and bitCount = 0 then
			iDone <= '1'; --read done
		end if;
		
		if state = write and arbLost = '1' then
			iDone <= '1'; --write done
			iErr <= '1'; --we lost the arbitration
			errType <= errArb;
		end if;
		
		if (state = write and sclCnt = 0 and subState = "11")
            or ((state = SAck or state = read) and subState = "01") then
			shiftBit <= '1';
		end if;
		
		if state = start then
			latchAddr <= '1';
		end if;
		
		if state = SAck and subState = "11" then --get the data byte for the next write
			latchData <= '1';
		end if;
		
end process;

NEXT_STATE_DECODE: process(state, busState, slaveWait, arbLost, STB_I, MSG_I, RST, subState, bitCount, int_Rst, dataByte, A_I, currAddr, rwBit, sclCnt, addrNData)
   begin
      nextState <= state;  --default is to stay in current state
   
      case state is
         when idle =>
            if STB_I = '1' and busState = busFree and RST = '0' then
               nextState <= start;
            elsif ATTEMPT_SLAVE_UNBLOCK and timeOutCnt = 0 then
                nextState <= stop;
            end if;
				
         when start =>
            if sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" then
						nextState <= write;
					end if;
				end if;
			
			when write =>
				if arbLost = '1' then
					nextState <= idle;
				elsif sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" and bitCount = 0 then
						nextState <= SAck;
					end if;
				end if;
			
			when SAck =>
				if sclCnt = 0 then
					if int_Rst = '1' or (subState = "11" and dataByte(0) = '1') then
						nextState <= stop;
					elsif subState = "11" then
						if addrNData = '1' then --if we have just sent the address, tx/rx the data too
							if rwBit = '1' then
								nextState <= read;
							else
								nextState <= write;
							end if;
						elsif STB_I = '1' then
							if MSG_I = '1' or currAddr /= A_I then
								nextState <= start;
							else
								if rwBit = '1' then
									nextState <= read;
								else
									nextState <= write;
								end if;
							end if;
						else
							nextState <= stop;
						end if;
					end if;
				end if;
				
         when stop =>
			 --bugfix: if device is driving SDA low (read transfer) we cannot send stop bit
			 --check the arbitration flag
				if subState = "10" and sclCnt = 0 and arbLost = '0' then
					nextState <= idle;
				end if;
				--stay here, if stop bit cannot be sent, pulse clock an retry
			
			when read =>
				if sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" and bitCount = 7 then --bitCount will underflow
						if STB_I = '1' then
							if MSG_I = '1' or currAddr /= A_I then
								nextState <= MNAckStart;
							else
								nextState <= MAck;
							end if;
						else
							nextState <= MNAckStop;
						end if;
					end if;
				end if;
			
			when MAck =>
				if sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" then
						nextState <= read;
					end if;
				end if;
			
			when MNAckStart =>
				if arbLost = '1' then
					nextState <= idle; -- arbitration lost, back off, no error because we got all the data
				elsif sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" then
						nextState <= start;
					end if;
				end if;
			
			when MNAckStop =>
				if arbLost = '1' then
					nextState <= idle; -- arbitration lost, back off, no error because we got all the data
				elsif sclCnt = 0 then
					if int_Rst = '1' then
						nextState <= stop;
					elsif subState = "11" then
						nextState <= stop;
					end if;
				end if;
				
         when others =>
            nextState <= idle;
      end case;
   end process;
   
   SDA <= 'Z' when rSDA = '1' else '0';
   SCL <= 'Z' when rSCL = '1' else '0';
end Behavioral;
