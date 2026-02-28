import struct
import csv
import os
import glob

class BinConverter:
    def __init__(self):

        self.TYPE_NULL   = 0
        self.TYPE_BOOL   = 1
        self.TYPE_INT    = 2
        self.TYPE_FLOAT  = 3
        self.TYPE_STRING = 4
        
        self.NULL_MARKER = -9223372036854775808

    def convert(self, target_path):

        abs_path = os.path.abspath(target_path)
        norm_path = os.path.normpath(abs_path)

        if os.path.isdir(norm_path):
            self._convert_directory(norm_path)
        elif os.path.isfile(norm_path):

            log_dir = os.path.dirname(norm_path)
            grandparent_dir = os.path.dirname(log_dir)
            output_folder = os.path.join(grandparent_dir, "Converted_Bin_Files")
            
            os.makedirs(output_folder, exist_ok=True)
            self._convert_single_file(norm_path, output_folder)
        else:
            print(f"Error: Path not found -> {target_path}")

    def _convert_directory(self, dir_path):
        print(f"Scanning directory: {dir_path}")
        
        bin_files = glob.glob(os.path.join(dir_path, "*.bin"))
        
        if not bin_files:
            print("No .bin files found in directory.")
            return

        parent_dir = os.path.dirname(dir_path) 
        output_folder = os.path.join(parent_dir, "Converted_Bin_Files")
        
        os.makedirs(output_folder, exist_ok=True)
        print(f"Output Folder Created: {output_folder}")

        print(f"Found {len(bin_files)} files. Starting conversion...\n")
        
        success_count = 0
        for b_file in bin_files:
            if "tlv.bin" in b_file:
                continue

            if self._convert_single_file(b_file, output_folder):
                success_count += 1
                
        print(f"\nBatch processing complete. {success_count}/{len(bin_files)} files converted.")

    def _convert_single_file(self, bin_path, output_folder):
        file_name = os.path.basename(bin_path)
        csv_name = file_name.replace(".bin", ".csv")
        output_path = os.path.join(output_folder, csv_name)

        print(f"--> Processing: {file_name}", end=" ... ")
        
        try:
            with open(bin_path, "rb") as f:
    
                magic = f.read(4)
                if magic != b"LOG1":
                    print(f"SKIPPED (Not a standard LOG1 file)")
                    return False
                
                f.read(1) 
                
                num_cols_bytes = f.read(2)
                if not num_cols_bytes:
                    print("SKIPPED (Empty file)")
                    return False
                num_cols = struct.unpack("<H", num_cols_bytes)[0]
                
                headers = []
                col_types = []

                for _ in range(num_cols):
                    # Name Length
                    len_raw = f.read(2)
                    if not len_raw: break
                    name_len = struct.unpack("<H", len_raw)[0]
                    # Name
                    name = f.read(name_len).decode("utf-8")
                    headers.append(name)
                    # Type ID
                    type_id_byte = f.read(1)
                    if not type_id_byte: break
                    col_types.append(int.from_bytes(type_id_byte, "little"))

                with open(output_path, "w", newline="", encoding="utf-8") as f_out:
                    writer = csv.writer(f_out)
                    writer.writerow(headers)
                    
                    while True:
                        row = []
                        try:
                            for t_id in col_types:
                                if t_id == self.TYPE_STRING:
                                    s_bytes = b""
                                    while True:
                                        char = f.read(1)
                                        if not char: raise EOFError 
                                        if char == b"\x00": break   
                                        s_bytes += char
                                    row.append(s_bytes.decode("utf-8", errors="replace"))

                                elif t_id == self.TYPE_FLOAT:
                                    chunk = f.read(8)
                                    if len(chunk) < 8: raise EOFError
                                    val = struct.unpack("<d", chunk)[0]
                                    row.append(val)

                                elif t_id == self.TYPE_INT:
                                    chunk = f.read(8)
                                    if len(chunk) < 8: raise EOFError
                                    val = struct.unpack("<q", chunk)[0]
                                    if val == self.NULL_MARKER:
                                        row.append("")
                                    else:
                                        row.append(val)

                                elif t_id == self.TYPE_BOOL:
                                    chunk = f.read(8)
                                    if len(chunk) < 8: raise EOFError
                                    val = struct.unpack("<q", chunk)[0]
                                    row.append(bool(val))
                                else:
                                    f.read(8) 
                                    row.append("ERR")

                            writer.writerow(row)
                        except (EOFError, struct.error):
                            break

            print("DONE")
            return True

        except Exception as e:
            print(f"FAILED ({e})")
            return False