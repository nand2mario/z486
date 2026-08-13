#!/usr/bin/env python3
"""Generate the synchronous x87 ESC/FOP command-decode ROM."""

from pathlib import Path


ACTIONS = {
    name: value
    for value, name in enumerate(
        (
            "NONE", "FNINIT", "FNCLEX", "FCHS", "FABS", "FXAM",
            "PUSH_CONST", "FSQRT", "FPTAN", "FPATAN", "TRIG",
            "FRNDINT", "FDECSTP", "FINCSTP", "TX_ENV", "TX_STATE",
            "RX_ENV", "RX_STATE", "ARITH", "FLD_ST", "FXCH", "FFREE",
            "FSTP_ST", "MEMORY_MATH", "LOAD", "STORE",
        )
    )
}

EXEC = {
    "FRNDINT": 0, "FLD_M32": 1, "FLD_M64": 2, "FILD": 3,
    "FST_M32": 4, "FST_M64": 5, "FIST": 6, "ADD": 7, "SUB": 8,
    "COMPARE": 9, "MUL": 10, "DIV": 11, "SQRT": 12, "TRANS": 13,
}

RX = {
    "NONE": 0, "CONTROL": 1, "M32": 2, "M64": 3, "M80": 4,
    "I16": 5, "I32": 6, "I64": 7, "ENV": 8, "STATE": 9,
}


def decode_key(fop: int) -> int:
    if ((fop >> 6) & 0x3) == 0x3:
        return fop
    return (fop & 0x700) | (fop & 0x38)


def register_form(fop: int) -> bool:
    return ((fop >> 6) & 0x3) == 0x3


def group(fop: int) -> int:
    return (fop >> 8) & 0x7


def operation(fop: int) -> int:
    return (fop >> 3) & 0x7


def is_addsub(fop: int) -> bool:
    return register_form(fop) and group(fop) in (0, 4, 6) and \
        operation(fop) in (0, 4, 5)


def is_compare_pop2(fop: int) -> bool:
    return fop in (0x2E9, 0x6D9)


def is_compare(fop: int) -> bool:
    return ((register_form(fop) and group(fop) == 0 and
             operation(fop) in (2, 3)) or
            (register_form(fop) and group(fop) == 5 and
             operation(fop) in (4, 5)) or
            is_compare_pop2(fop) or fop == 0x1E4)


def is_mul(fop: int) -> bool:
    return register_form(fop) and group(fop) in (0, 4, 6) and \
        operation(fop) == 1


def is_div(fop: int) -> bool:
    return register_form(fop) and group(fop) in (0, 4, 6) and \
        operation(fop) in (6, 7)


def is_memory_math(fop: int) -> bool:
    return not register_form(fop) and group(fop) in (0, 2, 4, 6)


def reads_status(fop: int) -> bool:
    return fop == 0x7E0 or (group(fop) == 5 and not register_form(fop) and
                            operation(fop) == 7)


def fields(fop: int) -> dict[str, int]:
    key = decode_key(fop)
    result = {
        "action": ACTIONS["NONE"], "exec": 0, "compare": 0, "quiet": 0,
        "write": 0, "dest_sti": 0, "needs_sti": 0, "reverse": 0,
        "pop": 0, "parameter": 0,
        "arithmetic": int(is_addsub(fop) or is_compare(fop) or is_mul(fop) or
                          is_div(fop) or is_memory_math(fop) or
                          fop in (0x1FA, 0x1F2, 0x1F3, 0x1FE, 0x1FF, 0x1FC)),
        "status": int(reads_status(fop) or key == 0x138),
    }

    exact = {
        0x3E3: "FNINIT", 0x3E2: "FNCLEX", 0x1E0: "FCHS",
        0x1E1: "FABS", 0x1E5: "FXAM", 0x1FA: "FSQRT",
        0x1F2: "FPTAN", 0x1F3: "FPATAN", 0x1FC: "FRNDINT",
        0x1F6: "FDECSTP", 0x1F7: "FINCSTP", 0x130: "TX_ENV",
        0x530: "TX_STATE", 0x120: "RX_ENV", 0x520: "RX_STATE",
    }
    if key in exact:
        result["action"] = ACTIONS[exact[key]]
        if key == 0x1FA:
            result["exec"] = EXEC["SQRT"]
        elif key in (0x1F2, 0x1F3):
            result["exec"] = EXEC["TRANS"]
        elif key == 0x1FC:
            result["exec"] = EXEC["FRNDINT"]
        return result
    if key in range(0x1E8, 0x1EF):
        result["action"] = ACTIONS["PUSH_CONST"]
        result["parameter"] = key - 0x1E8
        return result
    if key in (0x1FE, 0x1FF):
        result["action"] = ACTIONS["TRIG"]
        result["exec"] = EXEC["TRANS"]
        result["parameter"] = key & 1
        return result

    addsub = is_addsub(fop)
    compare = is_compare(fop)
    mul = is_mul(fop)
    div = is_div(fop)
    if addsub or compare or mul or div:
        op = operation(fop)
        result.update(
            action=ACTIONS["ARITH"],
            exec=(EXEC["DIV"] if div else EXEC["MUL"] if mul else
                  EXEC["COMPARE"] if compare else
                  EXEC["SUB"] if op in (4, 5) else EXEC["ADD"]),
            compare=int(compare), quiet=int(group(fop) == 5),
            write=int(addsub or mul or div), dest_sti=int(group(fop) != 0),
            needs_sti=int(fop != 0x1E4),
            reverse=int((addsub or div) and op in (5, 7)),
            pop=(2 if is_compare_pop2(fop) else
                 1 if ((group(fop) == 0 and op == 3) or
                       (group(fop) == 5 and op == 5) or
                       ((addsub or mul or div) and group(fop) == 6)) else 0),
        )
        return result

    if (fop & 0x7F8) == 0x1C0:
        result["action"] = ACTIONS["FLD_ST"]
    elif (fop & 0x7F8) == 0x1C8:
        result["action"] = ACTIONS["FXCH"]
    elif (fop & 0x7F8) == 0x5C0:
        result["action"] = ACTIONS["FFREE"]
    elif (fop & 0x7F8) == 0x5D8:
        result["action"] = ACTIONS["FSTP_ST"]
    elif is_memory_math(fop):
        op = operation(fop)
        result.update(
            action=ACTIONS["MEMORY_MATH"],
            exec=(EXEC["DIV"] if op in (6, 7) else EXEC["MUL"] if op == 1
                  else EXEC["COMPARE"] if op in (2, 3)
                  else EXEC["SUB"] if op in (4, 5) else EXEC["ADD"]),
            compare=int(op in (2, 3)), reverse=int(op in (5, 7)),
            pop=int(op == 3),
            parameter={0: RX["M32"], 2: RX["I32"], 4: RX["M64"],
                       6: RX["I16"]}[group(fop)],
        )
    elif group(fop) == 1 and operation(fop) == 0:
        result.update(action=ACTIONS["LOAD"], parameter=RX["M32"])
    elif group(fop) == 5 and operation(fop) == 0:
        result.update(action=ACTIONS["LOAD"], parameter=RX["M64"])
    elif group(fop) == 3 and operation(fop) == 5:
        result.update(action=ACTIONS["LOAD"], parameter=RX["M80"])
    elif group(fop) == 7 and operation(fop) == 0:
        result.update(action=ACTIONS["LOAD"], parameter=RX["I16"])
    elif group(fop) == 3 and operation(fop) == 0:
        result.update(action=ACTIONS["LOAD"], parameter=RX["I32"])
    elif group(fop) == 7 and operation(fop) == 5:
        result.update(action=ACTIONS["LOAD"], parameter=RX["I64"])
    elif group(fop) == 1 and operation(fop) == 5:
        result.update(action=ACTIONS["LOAD"], parameter=RX["CONTROL"])
    elif group(fop) in (1, 5) and operation(fop) in (2, 3):
        width = 0 if group(fop) == 1 else 1
        result.update(action=ACTIONS["STORE"],
                      parameter=(int(bool(fop & 8)) << 3) | (width << 1))
    elif group(fop) == 3 and operation(fop) == 7:
        result.update(action=ACTIONS["STORE"], parameter=(1 << 3) | (2 << 1))
    elif group(fop) in (7, 3) and operation(fop) in (2, 3):
        width = 0 if group(fop) == 7 else 1
        result.update(action=ACTIONS["STORE"],
                      parameter=(1 << 0) | (int(bool(fop & 8)) << 3) |
                                (width << 1))
    elif group(fop) == 7 and operation(fop) == 7:
        result.update(action=ACTIONS["STORE"],
                      parameter=(1 << 0) | (1 << 3) | (2 << 1))
    return result


def pack(fop: int) -> int:
    f = fields(fop)
    return (f["action"] | (f["exec"] << 5) | (f["compare"] << 9) |
            (f["quiet"] << 10) | (f["write"] << 11) |
            (f["dest_sti"] << 12) | (f["needs_sti"] << 13) |
            (f["reverse"] << 14) | (f["pop"] << 15) |
            (f["parameter"] << 17) | (f["arithmetic"] << 21) |
            (f["status"] << 22))


def main() -> None:
    root = Path(__file__).resolve().parent
    words = [pack(fop) for fop in range(2048)]
    assert all(word < (1 << 23) for word in words)
    svh = [
        "function automatic logic [22:0] x87_command_decode_word(",
        "    input logic [10:0] decode_address);",
        "    case (decode_address)",
    ]
    svh.extend(
        f"        11'h{address:03x}: return 23'h{word:06x};"
        for address, word in enumerate(words) if word
    )
    svh.extend(("        default: return 23'h000000;", "    endcase", "endfunction"))
    (root / "x87_command_decode.svh").write_text("\n".join(svh) + "\n")

    mif = [
        "WIDTH=23;", "DEPTH=2048;", "ADDRESS_RADIX=HEX;", "DATA_RADIX=HEX;",
        "CONTENT BEGIN",
    ]
    mif.extend(f"    {address:03X} : {word:06X};" for address, word in enumerate(words))
    mif.append("END;")
    (root / "x87_command_decode.mif").write_text("\n".join(mif) + "\n")


if __name__ == "__main__":
    main()
