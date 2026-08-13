package x87_pkg;

typedef enum logic [2:0] {
    X87_EMPTY,
    X87_ZERO,
    X87_NORMAL,
    X87_DENORMAL,
    X87_INFINITY,
    X87_NAN
} x87_class_t;

// Extended exponent range with binary64-class precision. GRS bits are kept
// separately so conversion and arithmetic can share one eventual rounder.
typedef struct packed {
    logic         sign;       // Numeric sign.
    logic [14:0]  exp;        // Extended-format biased exponent.
    logic [52:0]  sig;        // Explicit leading bit plus 52 fraction bits.
    logic         guard_bit;  // First discarded bit.
    logic         round_bit;  // Second discarded bit.
    logic         sticky_bit; // OR of all remaining discarded bits.
    x87_class_t   class_id;   // Registered special-value classification.
} x87_reg_t;

function automatic x87_reg_t x87_empty();
    x87_reg_t value;
    value = '0;
    value.class_id = X87_EMPTY;
    return value;
endfunction

function automatic x87_reg_t x87_zero(input logic sign);
    x87_reg_t value;
    value = '0;
    value.sign = sign;
    value.class_id = X87_ZERO;
    return value;
endfunction

function automatic x87_reg_t x87_one();
    x87_reg_t value;
    value = '0;
    value.exp = 15'h3fff;
    value.sig = {1'b1, 52'h0};
    value.class_id = X87_NORMAL;
    return value;
endfunction

function automatic x87_reg_t x87_indefinite();
    x87_reg_t value;
    value = '0;
    value.sign = 1'b1;
    value.exp = 15'h7fff;
    value.sig = {2'b11, 51'h0};
    value.class_id = X87_NAN;
    return value;
endfunction

function automatic logic [1:0] x87_tag(input x87_reg_t value);
    case (value.class_id)
        X87_EMPTY: x87_tag = 2'b11;
        X87_ZERO:  x87_tag = 2'b01;
        X87_NORMAL:x87_tag = 2'b00;
        default:   x87_tag = 2'b10;
    endcase
endfunction

function automatic x87_reg_t x87_from_m32(input logic [31:0] raw);
    x87_reg_t value;
    logic [7:0] ieee_exp;
    logic [22:0] frac;
    integer bit_index;
    integer highest;
    begin
        value = '0;
        value.sign = raw[31];
        ieee_exp = raw[30:23];
        frac = raw[22:0];

        if (ieee_exp == 0) begin
            if (frac == 0) begin
                value.class_id = X87_ZERO;
            end else begin
                highest = 0;
                for (bit_index = 0; bit_index < 23; bit_index = bit_index + 1)
                    if (frac[bit_index]) highest = bit_index;
                value.exp = 15'(16383 + highest - 149);
                value.sig = {30'h0, frac} << (52 - highest);
                value.class_id = X87_DENORMAL;
            end
        end else if (ieee_exp == 8'hff) begin
            value.exp = 15'h7fff;
            value.sig = {1'b1, frac, 29'h0};
            value.class_id = (frac == 0) ? X87_INFINITY : X87_NAN;
        end else begin
            value.exp = 15'(ieee_exp - 127 + 16383);
            value.sig = {1'b1, frac, 29'h0};
            value.class_id = X87_NORMAL;
        end
        return value;
    end
endfunction

function automatic x87_reg_t x87_from_m64(input logic [63:0] raw);
    x87_reg_t value;
    logic [10:0] ieee_exp;
    logic [51:0] frac;
    integer bit_index;
    integer highest;
    begin
        value = '0;
        value.sign = raw[63];
        ieee_exp = raw[62:52];
        frac = raw[51:0];

        if (ieee_exp == 0) begin
            if (frac == 0) begin
                value.class_id = X87_ZERO;
            end else begin
                highest = 0;
                for (bit_index = 0; bit_index < 52; bit_index = bit_index + 1)
                    if (frac[bit_index]) highest = bit_index;
                value.exp = 15'(16383 + highest - 1074);
                value.sig = {1'b0, frac} << (52 - highest);
                value.class_id = X87_DENORMAL;
            end
        end else if (ieee_exp == 11'h7ff) begin
            value.exp = 15'h7fff;
            value.sig = {1'b1, frac};
            value.class_id = (frac == 0) ? X87_INFINITY : X87_NAN;
        end else begin
            value.exp = {4'h0, ieee_exp} + 15'd15360;
            value.sig = {1'b1, frac};
            value.class_id = X87_NORMAL;
        end
        return value;
    end
endfunction

function automatic x87_reg_t x87_from_m80(input logic [79:0] raw);
    x87_reg_t value;
    begin
        value = '0;
        value.sign = raw[79];
        value.exp = raw[78:64];
        value.sig = raw[63:11];
        value.guard_bit = raw[10];
        value.round_bit = raw[9];
        value.sticky_bit = |raw[8:0];
        if (raw[78:64] == 0)
            value.class_id = (raw[63:0] == 0) ? X87_ZERO : X87_DENORMAL;
        else if (raw[78:64] == 15'h7fff)
            value.class_id = (raw[62:0] == 0) ? X87_INFINITY : X87_NAN;
        else
            value.class_id = X87_NORMAL;
        return value;
    end
endfunction

function automatic x87_reg_t x87_from_i64(input logic signed [63:0] raw);
    x87_reg_t value;
    logic [63:0] magnitude;
    integer bit_index;
    integer highest;
    begin
        if (raw == 0) begin
            value = x87_zero(1'b0);
        end else begin
            value = '0;
            value.sign = raw[63];
            magnitude = raw[63] ? (~raw + 64'd1) : raw;
            highest = 0;
            for (bit_index = 0; bit_index < 64; bit_index = bit_index + 1)
                if (magnitude[bit_index]) highest = bit_index;
            value.exp = 15'(16383 + highest);
            if (highest <= 52) begin
                value.sig = magnitude << (52 - highest);
            end else begin
                value.sig = magnitude >> (highest - 52);
                if (highest > 52) value.guard_bit = magnitude[highest - 53];
                if (highest > 53) value.round_bit = magnitude[highest - 54];
                if (highest > 54)
                    value.sticky_bit = |(magnitude << (64 - (highest - 54)));
            end
            value.class_id = X87_NORMAL;
        end
        return value;
    end
endfunction

function automatic logic [31:0] x87_to_m32(
    input x87_reg_t value,
    input logic [1:0] rounding_mode
);
    integer ieee_exp;
    integer drop;
    integer bit_index;
    logic [24:0] rounded;
    logic guard;
    logic sticky;
    logic discarded;
    logic increment;
    logic overflow_to_infinity;
    begin
        case (value.class_id)
            X87_ZERO:     x87_to_m32 = {value.sign, 31'h0};
            X87_INFINITY: x87_to_m32 = {value.sign, 8'hff, 23'h0};
            X87_NAN:      x87_to_m32 = {value.sign, 8'hff, 1'b1, value.sig[50:29]};
            X87_EMPTY:    x87_to_m32 = 32'hffc0_0000;
            default: begin
                ieee_exp = value.exp - 16383 + 127;
                drop = (ieee_exp <= 0) ? (30 - ieee_exp) : 29;
                rounded = (drop >= 53) ? 25'h0 : {1'b0, value.sig >> drop};
                guard = 1'b0;
                sticky = value.guard_bit || value.round_bit || value.sticky_bit;
                for (bit_index = 0; bit_index < 53; bit_index = bit_index + 1) begin
                    if (bit_index == (drop - 1))
                        guard = value.sig[bit_index];
                    if (bit_index < (drop - 1))
                        sticky = sticky || value.sig[bit_index];
                end
                discarded = guard || sticky;
                case (rounding_mode)
                    2'b00: increment = guard && (sticky || rounded[0]);
                    2'b01: increment = value.sign && discarded;
                    2'b10: increment = !value.sign && discarded;
                    default: increment = 1'b0;
                endcase
                rounded = rounded + increment;

                if (ieee_exp <= 0) begin
                    if (rounded[23])
                        x87_to_m32 = {value.sign, 8'h01, 23'h0};
                    else
                        x87_to_m32 = {value.sign, 8'h00, rounded[22:0]};
                end else begin
                    if (rounded[24]) begin
                        rounded = rounded >> 1;
                        ieee_exp = ieee_exp + 1;
                    end
                    if (ieee_exp >= 255) begin
                        case (rounding_mode)
                            2'b00: overflow_to_infinity = 1'b1;
                            2'b01: overflow_to_infinity = value.sign;
                            2'b10: overflow_to_infinity = !value.sign;
                            default: overflow_to_infinity = 1'b0;
                        endcase
                        x87_to_m32 = overflow_to_infinity
                                   ? {value.sign, 8'hff, 23'h0}
                                   : {value.sign, 8'hfe, 23'h7fffff};
                    end else begin
                        x87_to_m32 = {value.sign, ieee_exp[7:0], rounded[22:0]};
                    end
                end
            end
        endcase
    end
endfunction

function automatic logic [63:0] x87_to_m64(
    input x87_reg_t value,
    input logic [1:0] rounding_mode
);
    integer ieee_exp;
    integer drop;
    integer bit_index;
    logic [53:0] rounded;
    logic guard;
    logic sticky;
    logic discarded;
    logic increment;
    logic overflow_to_infinity;
    begin
        case (value.class_id)
            X87_ZERO:     x87_to_m64 = {value.sign, 63'h0};
            X87_INFINITY: x87_to_m64 = {value.sign, 11'h7ff, 52'h0};
            X87_NAN:      x87_to_m64 = {value.sign, 11'h7ff, 1'b1, value.sig[50:0]};
            X87_EMPTY:    x87_to_m64 = 64'hfff8_0000_0000_0000;
            default: begin
                ieee_exp = value.exp - 16383 + 1023;
                drop = (ieee_exp <= 0) ? (1 - ieee_exp) : 0;
                rounded = (drop >= 53) ? 54'h0 : {1'b0, value.sig >> drop};
                guard = (drop == 0) ? value.guard_bit : 1'b0;
                sticky = (drop == 0) ? (value.round_bit || value.sticky_bit)
                                     : (value.guard_bit || value.round_bit ||
                                        value.sticky_bit);
                for (bit_index = 0; bit_index < 53; bit_index = bit_index + 1) begin
                    if (bit_index == (drop - 1))
                        guard = value.sig[bit_index];
                    if (bit_index < (drop - 1))
                        sticky = sticky || value.sig[bit_index];
                end
                discarded = guard || sticky;
                case (rounding_mode)
                    2'b00: increment = guard && (sticky || rounded[0]);
                    2'b01: increment = value.sign && discarded;
                    2'b10: increment = !value.sign && discarded;
                    default: increment = 1'b0;
                endcase
                rounded = rounded + increment;

                if (ieee_exp <= 0) begin
                    if (rounded[52])
                        x87_to_m64 = {value.sign, 11'h001, 52'h0};
                    else
                        x87_to_m64 = {value.sign, 11'h000, rounded[51:0]};
                end else begin
                    if (rounded[53]) begin
                        rounded = rounded >> 1;
                        ieee_exp = ieee_exp + 1;
                    end
                    if (ieee_exp >= 2047) begin
                        case (rounding_mode)
                            2'b00: overflow_to_infinity = 1'b1;
                            2'b01: overflow_to_infinity = value.sign;
                            2'b10: overflow_to_infinity = !value.sign;
                            default: overflow_to_infinity = 1'b0;
                        endcase
                        x87_to_m64 = overflow_to_infinity
                                   ? {value.sign, 11'h7ff, 52'h0}
                                   : {value.sign, 11'h7fe, 52'hf_ffff_ffff_ffff};
                    end else begin
                        x87_to_m64 = {value.sign, ieee_exp[10:0], rounded[51:0]};
                    end
                end
            end
        endcase
    end
endfunction

function automatic logic [79:0] x87_to_m80(input x87_reg_t value);
    logic [63:0] significand;
    begin
        significand = {value.sig, 11'h0};
        case (value.class_id)
            X87_ZERO:     x87_to_m80 = {value.sign, 15'h0, 64'h0};
            X87_INFINITY: x87_to_m80 = {value.sign, 15'h7fff, 64'h8000_0000_0000_0000};
            X87_NAN:      x87_to_m80 = {value.sign, 15'h7fff, 2'b11, value.sig[50:0], 11'h0};
            X87_EMPTY:    x87_to_m80 = 80'hffff_c000_0000_0000_0000;
            default:      x87_to_m80 = {value.sign, value.exp, significand};
        endcase
    end
endfunction

function automatic logic x87_has_fraction(input x87_reg_t value);
    integer unbiased;
    integer discarded_bits;
    integer bit_index;
    logic discarded;
    begin
        if ((value.class_id != X87_NORMAL) &&
            (value.class_id != X87_DENORMAL)) begin
            return 1'b0;
        end
        unbiased = value.exp - 16383;
        discarded_bits = 52 - unbiased;
        discarded = value.guard_bit || value.round_bit || value.sticky_bit;
        for (bit_index = 0; bit_index < 53; bit_index = bit_index + 1)
            if (bit_index < discarded_bits)
                discarded = discarded || value.sig[bit_index];
        return discarded;
    end
endfunction

function automatic logic signed [63:0] x87_to_i64(
    input x87_reg_t value,
    input logic [1:0] rounding_mode
);
    logic [63:0] magnitude;
    logic [63:0] significand;
    logic half_bit;
    logic below_half;
    logic discarded;
    logic increment;
    integer unbiased;
    integer shift;
    integer bit_index;
    begin
        if (value.class_id == X87_ZERO) begin
            x87_to_i64 = 64'sd0;
        end else if ((value.class_id == X87_NORMAL) ||
                     (value.class_id == X87_DENORMAL)) begin
            unbiased = value.exp - 16383;
            significand = {11'h0, value.sig};
            shift = 52 - unbiased;
            if (unbiased < -11)
                magnitude = 64'h0;
            else if (unbiased <= 52)
                magnitude = significand >> shift;
            else if (unbiased < 64)
                magnitude = significand << (unbiased - 52);
            else
                magnitude = 64'h8000_0000_0000_0000;

            discarded = value.guard_bit || value.round_bit || value.sticky_bit;
            half_bit = 1'b0;
            below_half = value.guard_bit || value.round_bit || value.sticky_bit;
            for (bit_index = 0; bit_index < 53; bit_index = bit_index + 1) begin
                if (bit_index < shift)
                    discarded = discarded || value.sig[bit_index];
                if (bit_index == (shift - 1))
                    half_bit = value.sig[bit_index];
                if (bit_index < (shift - 1))
                    below_half = below_half || value.sig[bit_index];
            end

            case (rounding_mode)
                2'b00: increment = half_bit && (below_half || magnitude[0]);
                2'b01: increment = value.sign && discarded;  // toward -infinity
                2'b10: increment = !value.sign && discarded; // toward +infinity
                default: increment = 1'b0;                    // toward zero
            endcase
            if (increment)
                magnitude = magnitude + 64'd1;

            if (value.sign) begin
                if (magnitude > 64'h8000_0000_0000_0000)
                    x87_to_i64 = 64'sh8000_0000_0000_0000;
                else
                    x87_to_i64 = -$signed(magnitude);
            end else if (magnitude[63]) begin
                x87_to_i64 = 64'sh8000_0000_0000_0000;
            end else begin
                x87_to_i64 = $signed(magnitude);
            end
        end else begin
            x87_to_i64 = 64'sh8000_0000_0000_0000;
        end
    end
endfunction

endpackage
