LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.all;

-- Advanced controller offering a four distinct modes of interaction with the HEX LED display, as follows
--		1) Individual segment toggling
--		2) Shapes display mode
--		3) Left and right shift
--		4) Decimal display mode

ENTITY Alt7Seg IS
	PORT (
		io_addr	: IN STD_LOGIC_VECTOR(10 DOWNTO 0);
		io_data	: IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		io_write	: IN STD_LOGIC;
		resetn 	: IN STD_LOGIC;
		segments	: OUT STD_LOGIC_VECTOR(41 DOWNTO 0)
	);
END Alt7Seg;

ARCHITECTURE arch OF Alt7Seg IS
	TYPE MODE_TYPE IS ( m_segment, m_shape, m_shift, m_hex_digit, m_clear, m_none);

	-- Signal to track mode
	SIGNAL mode	: MODE_TYPE;
	
	-- Overall memory of peripheral
	SIGNAL segments_mem 		: STD_LOGIC_VECTOR(41 DOWNTO 0) := NOT "000000000000000000000000000000000000000000";
	
	-- Signals from register map
	SIGNAL seg_mask		: STD_LOGIC_VECTOR(6 DOWNTO 0) := NOT "0000000";
	SIGNAL shape_type		: STD_LOGIC_VECTOR(4 DOWNTO 0) := NOT "00000";
	SIGNAL hex_digit		: STD_LOGIC_VECTOR(3 DOWNTO 0) := NOT "0000";
	SIGNAL disp_sel		: STD_LOGIC_VECTOR(2 DOWNTO 0) := NOT "000";
	SIGNAL seg_mode		: STD_LOGIC := '0';
	SIGNAL shift_dir		: STD_LOGIC := '0';
	SIGNAL shift_mode		: STD_LOGIC := '0';
	
	-- Buffers to temporarily keep output
	SIGNAL shape_buffer		: STD_LOGIC_VECTOR(6 DOWNTO 0) := NOT "0000000";
	SIGNAL digit_buffer		: STD_LOGIC_VECTOR(6 DOWNTO 0) := NOT "0000000";
	SIGNAL shift_buffer		: STD_LOGIC_VECTOR(6 DOWNTO 0) := NOT "0000000";	-- Needs to be updated
	
	-- Internal signal for controlling when latching/update of memory happens
	SIGNAL update_signal		: STD_LOGIC := '0';
	
	
BEGIN
	-- Controls when to latch into segments_mem, only happens when io_write is asserted and mode is valid
	update_signal <= '1' WHEN (io_write = '1') AND (mode /= m_none) ELSE
		'0';

	-- Assign signals from io_data
	seg_mask <= io_data(6 DOWNTO 0);
	disp_sel <= io_data(9 DOWNTO 7);
	seg_mode <= io_data(10);
	shape_type <= io_data(4 DOWNTO 0);
	shift_dir <= io_data(7);
	shift_mode <= io_data(8);
	hex_digit <= io_data(3 DOWNTO 0);
						  
	-- Combinationally select mode based on io_addr
	WITH io_addr SELECT
		mode <= 	m_segment 	WHEN "00000000011",	-- io_addr=3
					m_shape		WHEN "00000000100",	-- io_addr=4
					m_shift		WHEN "00000000101",	-- io_addr=5
					m_hex_digit	WHEN "00000000110",	-- io_addr=6
					m_clear		WHEN "00000000111",	-- io_addr=7
					m_none	 	WHEN OTHERS;
					
	-- Combinationally compute digit based on io_data, store in digit_buffer
	WITH hex_digit SELECT
		digit_buffer <= "1000000" WHEN "0000",-- digit=0
							 "1111001" WHEN "0001", -- digit=1
							 "0100100" WHEN "0010", -- digit=2
							 "0110000" WHEN "0011", -- digit=3
							 "0011001" WHEN "0100", -- digit=4
							 "0010010" WHEN "0101", -- digit=5
							 "0000010" WHEN "0110", -- digit=6
							 "1111000" WHEN "0111", -- digit=7
							 "0000000" WHEN "1000", -- digit=8
							 "0010000" WHEN "1001", -- digit=9
							 "0001000" WHEN "1010", -- digit=A
							 "0000011" WHEN "1011", -- digit=B 
							 "1000110" WHEN "1100", -- digit=C
							 "0100001" WHEN "1101", -- digit=D
							 "0000110" WHEN "1110", -- digit=E
							 "0001110" WHEN "1111", -- digit=F
							 "0111111" WHEN OTHERS; -- digit=-
							 
	-- Combinationally compute shape based on io_data, store in shape_buffer
	WITH shape_type SELECT
		shape_buffer <= "0100011" WHEN "00000", -- Square Low
				          "0011100" WHEN "00001", -- Square High
				          "1100011" WHEN "00010", -- Square with open top low
				          "0011110" WHEN "00011", -- Square with open top high
				          "0100111" WHEN "00100", -- Square with open right low
				          "0011110" WHEN "00101", -- Square with open right high
				          "0110011" WHEN "00110", -- Square with open left low
				          "0111100" WHEN "00111", -- Square with open left high
				          "0101011" WHEN "01000", -- Square with open bottom low
				          "1011100" WHEN "01001", -- Square with open bottom high
				          "1000000" WHEN "01010", -- Rectangle
				          "1000001" WHEN "01011", -- Rectangle with open top
				          "1000110" WHEN "01100", -- Rectangle with open right
				          "1001000" WHEN "01101", -- Rectangle with open bottom
				          "1110000" WHEN "01110", -- Rectangle with open left
				          "1001111" WHEN "01111", -- Left Post
				          "1111001" WHEN "10000", -- Right Post
				          "1011110" WHEN "10001", -- Top left corner
				          "1111100" WHEN "10010", -- Top Right Corner
				          "1110011" WHEN "10011", -- Bottom Right Corner
				          "1100111" WHEN "10100", -- Bottom Left Corner
				          "0001001" WHEN "10101", -- X Shape
				          "0011000" WHEN "10110", -- Flag Left
				          "0001100" WHEN "10111", -- Flag Right
				          "0101001" WHEN "11000", -- Chair Left
				          "0001011" WHEN "11001", -- Chair Right
				          "0100100" WHEN "11010", -- Snake Left
				          "0010010" WHEN "11011", -- Snake Right
				          "0110110" WHEN "11100", -- Fence
				          "1111111" WHEN OTHERS;  -- invalid shape, so blank
	
	-- Process block to update display memory, which in turn updates display output
	PROCESS (update_signal, resetn)
	BEGIN
		IF (resetn = '0') THEN
			segments_mem <= (OTHERS => '1');
		ELSIF (RISING_EDGE(update_signal)) THEN	-- This ensures that memory is updated only when update signal first asserts
			CASE mode IS
				WHEN m_segment =>

					IF (seg_mode = '0') THEN
						CASE disp_sel IS
							WHEN "000" =>	-- 1st display from right
								segments_mem(6 DOWNTO 0) <= NOT seg_mask;
							WHEN "001" =>	-- 2nd display from right
								segments_mem(13 DOWNTO 7) <= NOT seg_mask;
							WHEN "010" =>	-- 3rd display from right
								segments_mem(20 DOWNTO 14) <= NOT seg_mask;
							WHEN "011" =>	-- 4th display from right
								segments_mem(27 DOWNTO 21) <= NOT seg_mask;
							WHEN "100" =>	-- 5th display from right
								segments_mem(34 DOWNTO 28) <= NOT seg_mask;
							WHEN "101" =>	-- 6th display from right
								segments_mem(41 DOWNTO 35) <= NOT seg_mask;
							WHEN OTHERS =>
								NULL;
						END CASE;
					ELSE
						CASE disp_sel IS
							WHEN "000" =>	-- 1st display from right
								segments_mem(6 DOWNTO 0) <= (NOT (segments_mem(6 DOWNTO 0) XOR seg_mask));
							WHEN "001" =>	-- 2nd display from right
								segments_mem(13 DOWNTO 7) <= (NOT ((NOT segments_mem(13 DOWNTO 7)) XOR seg_mask));
							WHEN "010" =>	-- 3rd display from right
								segments_mem(20 DOWNTO 14) <= (NOT ((NOT segments_mem(20 DOWNTO 14)) XOR seg_mask));
							WHEN "011" =>	-- 4th display from right
								segments_mem(27 DOWNTO 21) <= (NOT ((NOT segments_mem(27 DOWNTO 21)) XOR seg_mask));
							WHEN "100" =>	-- 5th display from right
								segments_mem(34 DOWNTO 28) <= (NOT ((NOT segments_mem(34 DOWNTO 28)) XOR seg_mask));
							WHEN "101" =>	-- 6th display from right
								segments_mem(41 DOWNTO 35) <= (NOT ((NOT segments_mem(41 DOWNTO 35)) XOR seg_mask));
							WHEN OTHERS =>
								NULL;
						END CASE;
					END IF;
					
				WHEN m_shift =>
				
					CASE io_data(1 DOWNTO 0) IS
						WHEN "00" => -- basic left shift
							segments_mem <= segments_mem(34 DOWNTO 0) & shift_buffer;
						WHEN "01" => -- basic right shift
							segments_mem <= shift_buffer & segments_mem(41 DOWNTO 7);
						WHEN "10" => -- circular left shift
							segments_mem <= segments_mem(34 DOWNTO 0) & segments_mem(41 DOWNTO 35);
						WHEN "11" => -- circular right shift
							segments_mem <= segments_mem(6 DOWNTO 0) & segments_mem(41 DOWNTO 7);
						WHEN OTHERS =>
							NULL;
					END CASE;
					
				WHEN m_hex_digit =>
				
					CASE io_data(9 DOWNTO 7) IS
						WHEN "000" =>
							segments_mem(6 DOWNTO 0) <= digit_buffer;
						WHEN "001" =>
							segments_mem(13 DOWNTO 7) <= digit_buffer;
						WHEN "010" =>
							segments_mem(20 DOWNTO 14) <= digit_buffer;
						WHEN "011" =>
							segments_mem(27 DOWNTO 21) <= digit_buffer;
						WHEN "100" =>
							segments_mem(34 DOWNTO 28) <= digit_buffer;
						WHEN "101" =>
							segments_mem(41 DOWNTO 35) <= digit_buffer;
						WHEN OTHERS =>
							NULL;
					END CASE;
						
				WHEN m_shape => 
					
					CASE disp_sel IS
						WHEN "000" =>
							segments_mem(6 DOWNTO 0) <= shape_buffer;
						WHEN "001" =>
							segments_mem(13 DOWNTO 7) <= shape_buffer;
						WHEN "010" =>
							segments_mem(20 DOWNTO 14) <= shape_buffer;
						WHEN "011" =>
							segments_mem(27 DOWNTO 21) <= shape_buffer;
						WHEN "100" =>
							segments_mem(34 DOWNTO 28) <= shape_buffer;
						WHEN "101" =>
							segments_mem(41 DOWNTO 35) <= shape_buffer;
						WHEN OTHERS =>
							NULL;
					END CASE;
				
				WHEN m_clear =>
					segments_mem <= (OTHERS => '1');
				WHEN OTHERS =>
					NULL;
			END CASE;
		END IF;
	END PROCESS;
	
	-- Propagate persistent memory as output
	segments <= segments_mem;
	
END arch;
