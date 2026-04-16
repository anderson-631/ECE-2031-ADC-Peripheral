-- LTC2308_ctrl.vhd
-- 
-- This module implements an SPI controller customized for an LTC2308 
-- Analog-to-Digital Converter (ADC), integrated as a SCOMP IO peripheral.
--
-- IO Address: 0xC0
--
-- OUT instruction: lower 6 bits of IO_DATA are loaded into the upper 6 bits
--                  of the SPI TX word (the ADC channel/config field), and a
--                  conversion is immediately triggered.
--
-- IN instruction:  returns the most recent 12-bit ADC result, sign-extended
--                  to 16 bits. Returns 0x8800 while a conversion is in
--                  progress or no valid result is available yet.
--
-- Generics:
--   CLK_DIV : Divides the main clock to generate the SCLK frequency.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library lpm;
use lpm.lpm_components.all;

entity LTC2308_ctrl is
	generic (
		-- Divides the main clock to generate SCLK frequency.
		-- Note that there's an additional factor of 2 because
		-- this CLK_DIV factor defines the rate at which SCLK
		-- will rise *and* fall.
		CLK_DIV : integer := 1 
	);
	port (
		-- Control and data for this device
		clk      : in  std_logic;
		nrst     : in  std_logic;
		busy     : out std_logic;
		
		-- SPI Physical Interface
		sclk     : out std_logic; -- Serial clock
		conv     : out std_logic; -- Conversion start control
		mosi     : out std_logic; -- Data out from this device, in to ADC
		miso     : in  std_logic; -- Data out from ADC, in to this device
		
		-- SCOMP IO ports
		IO_ADDR  : in    std_logic_vector(10 downto 0);
		IO_READ  : in    std_logic;
		IO_WRITE : in    std_logic;
		IO_DATA  : inout std_logic_vector(15 downto 0)
	);
end entity LTC2308_ctrl;

architecture internals of LTC2308_ctrl is

	-- State machine
	type state_type is (IDLE, CONV_PULSE, CONV_WAIT, TRANSFER, HOLD);
	signal state : state_type;
	
	-- Internal signals for clock generation
	signal clk_cnt   : integer range 0 to CLK_DIV;
	signal sclk_int  : std_logic;
	signal sclk_rise : std_logic;
	signal sclk_fall : std_logic;
	
	-- Internal signals for command/control
	signal bit_cnt   : integer range 0 to 12;
	signal wait_cnt  : integer range 0 to 200;
	
	-- TX data register.
	-- Upper 6 bits hold the ADC channel/config field written by SCOMP OUT.
	-- Lower 6 bits are always 0 (don't-care per LTC2308 datasheet).
	signal tx_data   : std_logic_vector(11 downto 0);
	signal tx_reg    : std_logic_vector(11 downto 0);
	signal rx_reg    : std_logic_vector(11 downto 0);
	
	-- Latched ADC result (12 bits), updated when a conversion completes.
	signal rx_data   : std_logic_vector(11 downto 0);

	-- Set high while a conversion is in flight; cleared when rx_data is latched.
	signal result_valid : std_logic;
	
	-- IO signals
	signal IO_EN     : std_logic;
	-- Full 16-bit word driven onto the IO bus (sign-extended result or sentinel).
	signal io_out    : std_logic_vector(15 downto 0);

begin

	-- Output assignment for the SPI Clock.
	-- An internal version is needed so that logic inside this device can be based
	-- on it (reading an output is not allowed in VHDL).
	sclk <= sclk_int;

	-------------------------------------------------------------------
	-- IO Address Decode
	-- Assert IO_EN only when the CPU is reading from address 0xC0.
	-------------------------------------------------------------------
	IO_EN <= '1' when (IO_ADDR = "00011000000" and IO_READ = '1') else '0';
	-- Note: IO_ADDR is 11 bits. 0xC0 = 192 decimal = 11000_0000 binary,
	-- which in 11 bits is "000_1100_0000".

	-------------------------------------------------------------------
	-- IO Output Mux
	-- Drive 0x8800 (sentinel / "no valid reading") while a conversion
	-- is in progress or no result has been captured since reset.
	-- Once a valid result is latched, drive it sign-extended to 16 bits.
	--
	-- 0x8800 is an impossible real ADC output:
	--   - Unipolar readings have 0x0 in the upper nibble.
	--   - Bipolar negative readings have 0xF in the upper nibble (sign extension).
	-- Therefore bit 15 high and bit 11 high is unreachable, making 0x8800 safe.
	-------------------------------------------------------------------
	io_out <= "0001000000000000" when result_valid = '0'
	     else "0000" & rx_data;
	--         ^ zero-extend across the upper 4 bits

	-- Use LPM function to create bidirectional I/O data bus.
	IO_BUS: lpm_bustri
	generic map (
		lpm_width => 16
	)
	port map (
		data     => io_out,
		enabledt => IO_EN,
		tridata  => IO_DATA
	);

	-------------------------------------------------------------------
	-- IO Write Process
	-- On an OUT instruction to 0xC0, capture the lower 6 bits of
	-- IO_DATA into the upper 6 bits of tx_data and trigger a conversion.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			-- Default: single-ended channel 0 (CH0, UNI, SLEEP=0, COM=0)
			-- Upper 6 bits = "100010" per LTC2308 datasheet Table 1
			tx_data      <= "100010000000";
			result_valid <= '0';
		elsif rising_edge(clk) then
			if IO_WRITE = '1' and IO_ADDR = "00011000000" then
				-- Interpret lower 3-bits of IO_DATA as an integer and assign correct configuration bits
				-- I.e. maps IO_DATA to channel number
				case to_integer(unsigned(IO_DATA(2 downto 0))) is
					 when 0 => tx_data(11 downto 8) <= "1000"; -- CH0: S/D=1 O/S=0 S1=0 S0=0
					 when 1 => tx_data(11 downto 8) <= "1100"; -- CH1: S/D=1 O/S=1 S1=0 S0=0
					 when 2 => tx_data(11 downto 8) <= "1001"; -- CH2: S/D=1 O/S=0 S1=0 S0=1
					 when 3 => tx_data(11 downto 8) <= "1101"; -- CH3: S/D=1 O/S=1 S1=0 S0=1
					 when 4 => tx_data(11 downto 8) <= "1010"; -- CH4: S/D=1 O/S=0 S1=1 S0=0
					 when 5 => tx_data(11 downto 8) <= "1110"; -- CH5: S/D=1 O/S=1 S1=1 S0=0
					 when 6 => tx_data(11 downto 8) <= "1011"; -- CH6: S/D=1 O/S=0 S1=1 S0=1
					 when 7 => tx_data(11 downto 8) <= "1111"; -- CH7: S/D=1 O/S=1 S1=1 S0=1
					 when others => tx_data(11 downto 8) <= "1000"; -- default to CH0
				end case;
				tx_data(7 downto 0) <= "10000000"; -- UNI=1, SLEEP=0, don't-cares
				-- Explicity reassign lower byte for robustness
				result_valid <= '0'; -- Invalidate previous result
			end if;

			-- Latch result valid flag when a fresh rx_data is committed (see
			-- Data Process below). We detect the HOLD->IDLE transition via
			-- the state machine: rx_data is latched in HOLD on sclk_fall,
			-- so set result_valid one cycle later.
			if state = HOLD and sclk_fall = '1' then
				result_valid <= '1';
			end if;
		end if;
	end process;

	-------------------------------------------------------------------
	-- Controlling Process
	-- Handles the start signal (now internal: triggered by IO_WRITE),
	-- busy flag, wait timer, and counts the 12 bits as they are
	-- transmitted.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			state    <= IDLE;
			bit_cnt  <= 0;
			wait_cnt <= 0;
			conv     <= '0';
			busy     <= '0';
		elsif rising_edge(clk) then
			case state is
				when IDLE =>
					conv <= '0';
					busy <= '0';
					-- Trigger immediately when SCOMP writes new config.
					if IO_WRITE = '1' and IO_ADDR = "00011000000" then
						state <= CONV_PULSE;
						conv  <= '1'; -- One-cycle CONVST pulse
						busy  <= '1';
					end if;

				when CONV_PULSE =>
					conv     <= '0';  -- Bring CONVST low to keep ADC awake
					wait_cnt <= 40;   -- Wait ~1.6 us at 25 MHz for t_conv
					state    <= CONV_WAIT;

				when CONV_WAIT =>
					if wait_cnt = 0 then
						state   <= TRANSFER;
						bit_cnt <= 12 - 1;
					else
						wait_cnt <= wait_cnt - 1;
					end if;

				when TRANSFER =>
					-- Decrement bit count on each rising SCLK edge
					if sclk_rise = '1' then
						bit_cnt <= bit_cnt - 1;
						if bit_cnt = 0 then
							state <= HOLD;
						end if;
					end if;
					  
				when HOLD =>
					conv <= '0';
					busy <= '0';
					-- Return to IDLE unconditionally; re-trigger only on next OUT
					state <= IDLE;
					  
			end case;
		end if;
	end process;

	-------------------------------------------------------------------
	-- Clock Generation Process
	-- Divides the system clock for SCLK and generates flag signals to 
	-- control other parts of the system.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			clk_cnt   <= 0;
			sclk_int  <= '0';
			sclk_rise <= '0';
			sclk_fall <= '0';
		elsif rising_edge(clk) then
			-- Note that because this is in a process, these values
			-- can be "overridden" by lines of code lower in the block.
			sclk_rise <= '0';
			sclk_fall <= '0';
			
			if state = TRANSFER then
				clk_cnt <= clk_cnt + 1;
				if clk_cnt = CLK_DIV - 1 then
					clk_cnt <= 0;
					
					sclk_int <= not sclk_int; -- Toggle SCLK
					if sclk_int = '0' then
						sclk_rise <= '1'; -- SCLK is transitioning 0 -> 1
					else
						sclk_fall <= '1'; -- SCLK is transitioning 1 -> 0
					-- If those IF conditions seem backwards to you, you're
					-- thinking like software instead of thinking like hardware.
					end if;
				end if;
			else
				clk_cnt  <= 0;
				sclk_int <= '0'; -- Ensure SCLK idles low
			end if;
		end if;
	end process;

	-------------------------------------------------------------------
	-- Data Process
	-- Manages the TX and RX shift registers.
	-- Samples MISO on rising edges and shifts MOSI out on falling edges.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			tx_reg  <= (others => '0');
			rx_reg  <= (others => '0');
			rx_data <= (others => '0');
			mosi    <= '0';
		elsif rising_edge(clk) then
			
			if state = IDLE then
				-- Load tx_data immediately when a write arrives so the first
				-- MOSI bit is already set up before CONV_PULSE ends.
				if IO_WRITE = '1' and IO_ADDR = "00011000000" then
					tx_reg <= tx_data;
					mosi   <= tx_data(11); -- MSB of the 6-bit config field
				end if;
				 
			elsif state = TRANSFER then
				-- Sample MISO on rising edges
				if sclk_rise = '1' then
					rx_reg <= rx_reg(10 downto 0) & miso;
				end if;
				 
				-- Shift MOSI on falling edges
				if sclk_fall = '1' then
					tx_reg <= tx_reg(10 downto 0) & '0';
					mosi   <= tx_reg(10); -- Put the next MSB onto the line
				end if;
				 
			elsif state = HOLD then
				-- Latch received data on the final SCLK falling edge.
				if sclk_fall = '1' then
					rx_data <= rx_reg;
				end if;
			end if;
			
		end if;
	end process;

end architecture internals;