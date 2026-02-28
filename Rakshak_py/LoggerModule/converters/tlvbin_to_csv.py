import struct
import csv
import os
import glob

class TLVBinConverter:
    def __init__(self):
        
        self.TYPE_NULL   = 0
        self.TYPE_BOOL   = 1
        self.TYPE_INT    = 2
        self.TYPE_FLOAT  = 3
        self.TYPE_STRING = 4
        self.TYPE_BYTES  = 5
        self.TYPE_NONE   = 6
        
        self.FIELD_DEF_MARKER = 0x01

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
        
        files = glob.glob(os.path.join(dir_path, "*.tlv"))
        files = list(set(files)) 

        if not files:
            print("No .tlv  files found.")
            return

        parent_dir = os.path.dirname(dir_path) 
        output_folder = os.path.join(parent_dir, "Converted_Bin_Files")
        os.makedirs(output_folder, exist_ok=True)
        print(f"Output Folder Created: {output_folder}")

        print(f"Found {len(files)} files. Starting conversion...\n")
        
        success_count = 0
        for f_path in sorted(files):
            if self._convert_single_file(f_path, output_folder):
                success_count += 1
                
        print(f"\nBatch processing complete. {success_count}/{len(files)} files converted.")

    def _convert_single_file(self, file_path, output_folder):
        file_name = os.path.basename(file_path)
        
        name_part = file_name.replace(".tlv", "")
        csv_name = f"{name_part}.csv"
        output_path = os.path.join(output_folder, csv_name)

        print(f"--> Processing: {file_name}", end=" ... ")
        
        try:
            with open(file_path, "rb") as f:
                magic = f.read(4)
                if magic != b"TLV1":
                    print(f"SKIPPED (Not a TLV1 file)")
                    return False
                

                version = int.from_bytes(f.read(1), "little")
                field_count = int.from_bytes(f.read(1), "little")
                
                headers = []
                for i in range(field_count):
                    marker = int.from_bytes(f.read(1), "little")
                    if marker != self.FIELD_DEF_MARKER:
                         print(f"Corrupt Header at field {i}")
                         return False
                         
                    name_len = int.from_bytes(f.read(2), "little")
                    name = f.read(name_len).decode("utf-8")
                    headers.append(name)


                with open(output_path, "w", newline="", encoding="utf-8") as f_out:
                    writer = csv.writer(f_out)
                    writer.writerow(headers)

                    while True:

                        size_raw = f.read(2)
                        if not size_raw: break
                        record_size = int.from_bytes(size_raw, "little")
                        
                        record_buf = f.read(record_size)
                        if len(record_buf) < record_size: break

                        pos = 0
                        row = []
                        try:
                            while pos < record_size:
                                t = record_buf[pos]
                                pos += 1
                                
                                l = int.from_bytes(record_buf[pos:pos+2], "little")
                                pos += 2
                                
                                value_bytes = record_buf[pos:pos+l]
                                pos += l
                                
                                row.append(self._decode_value(t, value_bytes))
                            writer.writerow(row)
                        except Exception:
                            break
                            
            print("DONE")
            return True

        except Exception as e:
            print(f"FAILED ({e})")
            return False

    def _decode_value(self, t, raw):

        try:
            if t == self.TYPE_NONE:   return ""
            if t == self.TYPE_BOOL:   return bool(raw[0]) if raw else False
            if t == self.TYPE_INT:    return struct.unpack("<q", raw)[0]
            if t == self.TYPE_FLOAT:  return struct.unpack("<d", raw)[0]
            if t == self.TYPE_STRING: return raw.decode("utf-8", errors="replace")
            if t == self.TYPE_BYTES:  return raw.hex()
            return f"ERR_TYPE_{t}"
        except Exception:
            return "ERR_DECODE"
