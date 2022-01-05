----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12/28/2021 08:06:32 PM
-- Design Name: 
-- Module Name: SensorCTRL - Behavioral
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


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.math_real.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity SensorCTRL is
	Generic (CLOCKFREQ : natural := 100); -- input CLK frequency in MHz
    Port (clk : in STD_LOGIC;
          rst : in STD_LOGIC;
          
            SCL : inout STD_LOGIC;
            SDA : inout STD_LOGIC;
            
            out_temperature : out STD_LOGIC_VECTOR(12 downto 0);
            out_ready : out STD_LOGIC;
            out_error : out STD_LOGIC);
end SensorCTRL;

architecture Behavioral of SensorCTRL is
------------ I2C initialization vector ------------
	constant IRD : std_logic := '1'; -- read
    constant IWR : std_logic := '0'; -- write
    
    constant ADT7420_ADDR : std_logic_vector(7 downto 1)     := "1001011";  -- TWI Slave Address
    constant ADT7420_RID : std_logic_vector(7 downto 0)      := x"0B";      -- ID Register Address for the ADT7420
    constant ADT7420_RRESET : std_logic_vector(7 downto 0)   := x"2F";      -- Software Reset Register
    constant ADT7420_RTEMP : std_logic_vector(7 downto 0)    := x"00";      -- Temperature Read MSB Address
    constant ADT7420_ID : std_logic_vector(7 downto 0)       := x"CB";      -- ADT7420 Manufacturer ID
    
    constant DELAY : NATURAL := 1; -- in ms
    constant DELAY_CYCLES : NATURAL := natural(ceil(real(DELAY*1000*CLOCKFREQ)));
    constant RETRY_COUNT : NATURAL := 10;
---------------------------------------------------


    constant NO_OF_INIT_VECTORS : natural := 3;
    constant DATA_WIDTH : integer := 1 + 8 + 8;
    constant ADDR_WIDTH : natural := natural(ceil(log(real(NO_OF_INIT_VECTORS), 2.0)));    
    
    type state_type is (idle, initReg, initData, retry, readTempR, readTempD1, readTempD2, errorState); 
    signal state, nextState : state_type;

	type TempSensInitMap_type is array (0 to NO_OF_INIT_VECTORS-1) of std_logic_vector(DATA_WIDTH-1 downto 0);
	signal TempSensInitMap: TempSensInitMap_type := (
		  IRD & x"0B" & x"CB", -- Read ID R[0x0B]=0xCB
		  IWR & x"2F" & x"00", -- Reset R[0x2F]=don't care
		  IRD & x"0B" & x"CB"); -- Read ID R[0x0B]=0xCB
		  
    signal initWord: std_logic_vector (DATA_WIDTH-1 downto 0);
        signal initA : natural range 0 to NO_OF_INIT_VECTORS := 0; --init vector index
        signal initEn : std_logic;

    --Two-Wire Controller signals
	signal twiMsg, twiStb, twiDone, twiErr : std_logic;
	signal twiDi, twiDo, twiAddr : std_logic_vector(7 downto 0);

	--Wait counter used between retry attempts
	signal waitCnt : natural range 0 to DELAY_CYCLES := DELAY_CYCLES;
	signal waitCntEn : std_logic;
	
	--Retry counter to count down attempts to get rid of a bus error
    signal retryCnt : natural range 0 to RETRY_COUNT := RETRY_COUNT;
    signal retryCntEn : std_logic;    
    
    -- Temporary register to store received data
    signal tempReg : std_logic_vector(15 downto 0) := (others => '0');
    
    -- Flag indicating that a new temperature data is available
   signal fReady : boolean := false;
   
begin
    out_temperature <= tempReg(15 downto 3);
    
    out_ready <= '1' when fReady else '0';
    out_error <= '1' when state = errorState else '0';
    
---------- TWI controller ----------
    TWI : entity work.TWI_controller 
    generic map(ATTEMPT_SLAVE_UNBLOCK => true, CLKFREQ => 100)
    port map (clk  => clk,
            rst => rst,
            MSG_I => twiMsg,
            STB_I => twiStb,
            A_I => twiAddr,
            D_I => twiDi,
            D_O => twiDo,
            done_signal => twiDone,
            error_signal => twiErr,
            errortype => open,
            SDA => SDA,
            SCL => SCL);
            
---------- init RAM map ----------
    initWord <= TempSensInitMap(initA);

InitA_CNT: process (clk) 
    begin
        if Rising_Edge(clk) then
            if state = idle or initA = NO_OF_INIT_VECTORS then
                initA <= 0;
            elsif (initEn = '1') then
                initA <= initA + 1;
            end if;
        end if;
end process;

---------- Delay and Retry Counters ----------
Wait_CNT: process (clk) 
    begin
        if Rising_Edge(clk) then
            if waitCntEn = '0' then
                waitCnt <= DELAY_CYCLES;
            else
                waitCnt <= waitCnt - 1;
            end if;
        end if;
end process;

Retry_CNT: process (clk) 
    begin
        if Rising_Edge(clk) then
            if state = idle then
                retryCnt <= RETRY_COUNT;
            elsif retryCntEn = '1' then
                retryCnt <= retryCnt - 1;
            end if;
        end if;
end process;

----------  temp reg ----------
TempRegProc: process (clk) 
    variable temp : STD_LOGIC_VECTOR(7 downto 0);
    begin
        if Rising_Edge(clk) then
            if state = readTempD1 and twiDone = '1' and twiErr = '0' then
                temp := twiDo;
            end if;
            
            if state = readTempD2 and twiDone = '1' and twiErr = '0' then
                tempReg <= temp & twiDo;
            end if;
        end if;
end process;

----------  R E A D Y ----------
RdyFlag: process(clk) 
    begin
        if Rising_Edge(clk) then
            if state = idle or state = errorState then
                fReady <= false;
            elsif state = readTempD2 and twiDone = '1' and twiErr = '0' then
                fReady <= true;
            end if;
        end if;
end process;	

---------- Init FSM & cont temp read ----------
SyncProc: process (clk)
   begin
      if clk'event and clk = '1' then
         if rst = '1' then
            state <= idle;
         else
            state <= nextState;
         end if;        
      end if;
end process;

OutputDec: process (state, initWord, twiDone, twiErr, twiDo, retryCnt, waitCnt, initA)
    begin
        twiStb <= '0';                   -- send/receive strobe
        twiMsg <= '0';                   -- new transfer request
        waitCntEn <= '0';                -- wait countdown enable
        twiDi <= "--------";             -- byte to send
        twiAddr <= ADT7420_ADDR & '0';   -- I2C device address with R/W bit
        initEn <= '0';                   -- increase init map address
        retryCntEn <= '0';               -- retry countdown enable
    
        case state is
            when idle =>            -- nothing happens lmao
            
            when initReg =>         -- sends the register address from the current init vector
                twiStb <= '1';
                twiMsg <= '1';
                twiAddr(0) <= IWR;
                twiDi <= initWord(15 downto 8);
        
            when initData =>        -- sends the data byte from the current init vector
                twiStb <= '1';
                twiAddr(0) <= initWord(initWord'high);
                twiDi <= initWord(7 downto 0);
                if twiDone = '1'
                    and (twiErr = '0' or (initWord(16) = IWR and initWord(15 downto 8) = ADT7420_RRESET))
                    and (initWord(initWord'high) = IWR or twiDo = initWord(7 downto 0)) then
                       initEn <= '1';
                end if;
                
            when retry =>           -- in case of an I2C error during initialization
                if retryCnt /= 0 then                
                   waitCntEn <= '1';
                   if waitCnt = 0 then
                       retryCntEn <= '1';
                   end if;
                end if;
    
            when readTempR =>       -- sends the temperature register address
                twiStb <= '1';
                twiMsg <= '1';
                twiDi <= ADT7420_RTEMP;
                twiAddr(0) <= IWR;
                               
            when readTempD1 =>      -- reads the temperature MSB
                twiStb <= '1';
                twiAddr(0) <= IRD;
                
            when readTempD2 =>      -- reads the temperature LSB
                twiStb <= '1';
                twiAddr(0) <= IRD;
    
            when errorState =>      -- in case of an I2C error during temperature poll
                null; --stay here
        end case;
end process;

NextStateDec: process (state, twiDone, twiErr, initWord, twiDo, retryCnt, waitCnt)
    begin
        nextState <= state;  -- default - stay in current state
        
        case state is
            when idle => nextState <= initReg;
        
            when initReg =>
                if twiDone = '1' then
                    if twiErr = '1' then
                        nextState <= retry;
                    else
                        nextState <= initData;
                    end if;
                end if;
        
            when initData =>
                if twiDone = '1' then
                    if twiErr = '1' then
                        nextState <= retry;
                    else
                        if initWord(initWord'high) = IRD and twiDo /= initWord(7 downto 0) then
                            nextState <= retry;
                        elsif initA = NO_OF_INIT_VECTORS-1 then
                            nextState <= readTempR;
                        else
                            nextState <= initReg;
                        end if;
                    end if;
                end if;		
                        
            when retry =>
                if retryCnt = 0 then
                    nextState <= errorState;
                elsif waitCnt = 0 then
                    nextState <= initReg; --new retry attempt
                end if;
        
            when readTempR =>
                if twiDone = '1' then
                    if twiErr = '1' then
                        nextState <= errorState;
                    else
                        nextState <= readTempD1;
                    end if;
                end if;
                
            when readTempD1 =>
                if twiDone = '1' then
                    if twiErr = '1' then
                        nextState <= errorState;
                    else
                        nextState <= readTempD2;
                    end if;
                end if;
                    
            when readTempD2 =>
                if twiDone = '1' then
                    if twiErr = '1' then
                        nextState <= errorState;
                    else
                        nextState <= readTempR;
                    end if;
                end if;
        
            when errorState => null; --stay
    
            when others => nextState <= idle;
        end case;      
end process;

end Behavioral;


