#!/usr/bin/env python3
"""
Monoprice Voxel GX File Converter
Converts STL/gcode/bgcode files to GX format with dimension validation for
150x150x150mm build area.

Usage:
    python voxel_converter.py <input_file>

Supported inputs: .stl, .gcode, .bgcode (binary gcode), .gx, .g
"""

import sys
import struct
import re
import zlib
from pathlib import Path
from typing import Tuple, Optional
import subprocess


# ---------------------------------------------------------------------------
# Binary G-code (bgcode) support
#
# bgcode is the PrusaSlicer / libbgcode container format. Layout:
#
#   File header:  magic 'GCDE' (4) | version u32 | checksum_type u16
#   Then repeated blocks:
#       type u16 | compression u16 | uncompressed_size u32
#       [compressed_size u32]   -- only when compression != 0
#       parameters             -- 6 bytes for thumbnails, else 2 bytes (encoding u16)
#       data                   -- compressed_size bytes if compressed, else uncompressed
#       [crc32 u32]            -- only when checksum_type == 1
#
#   Block types:   0 file-meta, 1 gcode, 2 slicer-meta, 3 printer-meta,
#                  4 print-meta, 5 thumbnail
#   Compression:   0 none, 1 deflate (zlib), 2/3 heatshrink
#   GCode encoding: 0 none, 1 MeatPack, 2 MeatPack w/ comments
# ---------------------------------------------------------------------------

BGCODE_MAGIC = b'GCDE'

_MP_LOOKUP = "0123456789. \nGX\x00"  # MeatPack 4-bit -> char table (index 15 = literal)
_MP_CMD_BYTE = 0xFF
_MP_CMD_DISABLE_NO_SPACES = 246
_MP_CMD_ENABLE_NO_SPACES = 247
_MP_CMD_QUERY = 248
_MP_CMD_RESET = 249
_MP_CMD_DISABLE_PACKING = 250
_MP_CMD_ENABLE_PACKING = 251
_MP_CMD_TOGGLE_PACKING = 253


def meatpack_decode(data: bytes, packing_default: bool = False) -> bytes:
    """Decode a MeatPack byte stream back to plain gcode text.

    MeatPack squeezes the common gcode characters (digits, '.', ' ', '\\n', 'G',
    'X') into 4-bit nibbles, two per byte. A nibble of 0b1111 means "this
    character is not packable, read the next full byte literally". The byte
    0xFF is the command escape: 0xFF 0xFF <cmd> toggles packing / no-spaces
    modes, while a lone 0xFF marks a byte whose two nibbles are both literal.
    """
    out = bytearray()
    packing = packing_default
    no_spaces = False
    ff_count = 0            # consecutive 0xFF bytes seen
    await_cmd = False       # next byte is a command code
    pending_literals = 0    # literal full-width bytes still expected
    deferred_char = None    # resolved char to emit after a pending literal

    def lookup(nib: int) -> str:
        if no_spaces and nib == 11:
            return 'E'
        return _MP_LOOKUP[nib]

    for b in data:
        if b == _MP_CMD_BYTE:
            ff_count += 1
            if ff_count == 2:
                await_cmd = True
                ff_count = 0
            continue

        if await_cmd:
            await_cmd = False
            if b == _MP_CMD_ENABLE_PACKING:
                packing = True
            elif b == _MP_CMD_DISABLE_PACKING:
                packing = False
            elif b == _MP_CMD_TOGGLE_PACKING:
                packing = not packing
            elif b == _MP_CMD_RESET:
                packing = False
                no_spaces = False
            elif b == _MP_CMD_ENABLE_NO_SPACES:
                no_spaces = True
            elif b == _MP_CMD_DISABLE_NO_SPACES:
                no_spaces = False
            # _MP_CMD_QUERY and anything unknown: ignore
            continue

        if ff_count == 1:
            # Lone 0xFF preceded this byte: "both nibbles literal" marker;
            # two literal bytes follow and this is the first.
            ff_count = 0
            out.append(b)
            pending_literals = 1
            continue

        if pending_literals > 0:
            out.append(b)
            pending_literals -= 1
            if pending_literals == 0 and deferred_char is not None:
                out.append(ord(deferred_char))
                deferred_char = None
            continue

        if not packing:
            out.append(b)
            continue

        low, high = b & 0x0F, (b >> 4) & 0x0F
        if low != 0x0F and high != 0x0F:
            out.append(ord(lookup(low)))
            out.append(ord(lookup(high)))
        elif low != 0x0F:  # high nibble literal, comes next
            out.append(ord(lookup(low)))
            pending_literals = 1
        elif high != 0x0F:  # low nibble literal, comes next; then known char
            pending_literals = 1
            deferred_char = lookup(high)
        else:
            pending_literals = 2  # unreachable (that byte would be 0xFF)

    return bytes(out)


def _bgcode_decompress(payload: bytes, compression: int) -> bytes:
    """Inflate a bgcode block payload."""
    if compression == 0:
        return payload
    if compression == 1:
        return zlib.decompress(payload)
    if compression in (2, 3):
        raise ValueError(
            "This bgcode uses Heatshrink compression, which this script cannot "
            "decode. Re-export it with 'No compression' (or Deflate), or run it "
            "through PrusaSlicer / libbgcode first."
        )
    raise ValueError(f"Unknown bgcode compression type: {compression}")


def decode_bgcode_to_text(data: bytes) -> str:
    """Extract concatenated plain-text gcode from a bgcode file's bytes."""
    if data[:4] != BGCODE_MAGIC:
        raise ValueError("Not a binary gcode file (missing 'GCDE' magic).")

    checksum_type = struct.unpack_from('<H', data, 8)[0]
    pos, n = 10, len(data)
    parts = []

    while pos + 8 <= n:
        btype, compression, uncomp_size = struct.unpack_from('<HHI', data, pos)
        pos += 8

        if compression != 0:
            comp_size = struct.unpack_from('<I', data, pos)[0]
            pos += 4
        else:
            comp_size = uncomp_size

        param_len = 6 if btype == 5 else 2
        params = data[pos:pos + param_len]
        pos += param_len

        payload = data[pos:pos + comp_size]
        pos += comp_size

        if checksum_type == 1:
            pos += 4  # skip trailing CRC32

        if btype != 1:  # only GCode blocks carry movement commands
            continue

        raw = _bgcode_decompress(payload, compression)
        encoding = struct.unpack_from('<H', params, 0)[0] if len(params) >= 2 else 0

        if encoding == 0:
            text = raw.decode('utf-8', 'ignore')
        elif encoding in (1, 2):
            decoded = meatpack_decode(raw)
            if b'G1' not in decoded and b'G0' not in decoded:
                # Stream may not have opened with an explicit "enable packing".
                decoded = meatpack_decode(raw, packing_default=True)
            text = decoded.decode('utf-8', 'ignore')
        else:
            raise ValueError(f"Unsupported bgcode gcode encoding: {encoding}")

        parts.append(text)

    if not parts:
        raise ValueError("No GCode blocks found in bgcode file.")
    return ''.join(parts)


class VoxelConverter:
    # Monoprice Voxel build area dimensions (mm)
    BUILD_AREA = (150.0, 150.0, 150.0)
    
    # Known GX binary header magic bytes for Monoprice Voxel
    GX_HEADER = b'\x00\x00\x00\x00'  # Placeholder - actual header may vary
    
    def __init__(self, input_file: str):
        self.input_path = Path(input_file)
        self.validate_input_exists()
        self.file_ext = self.input_path.suffix.lower()
    
    def validate_input_exists(self):
        """Check if input file exists."""
        if not self.input_path.exists():
            raise FileNotFoundError(f"File not found: {self.input_path}")
        if not self.input_path.is_file():
            raise ValueError(f"Not a file: {self.input_path}")
    
    def validate_extension(self) -> bool:
        """Validate file extension is STL, gcode, bgcode, GX, or G."""
        valid_exts = {'.stl', '.gcode', '.bgcode', '.gx', '.g'}
        if self.file_ext not in valid_exts:
            raise ValueError(
                f"Invalid file extension: {self.file_ext}. "
                f"Supported: {', '.join(sorted(valid_exts))}"
            )
        return True

    def is_bgcode(self) -> bool:
        """True if the input is binary gcode (by extension or 'GCDE' magic)."""
        if self.file_ext == '.bgcode':
            return True
        try:
            with self.input_path.open('rb') as fh:
                return fh.read(4) == BGCODE_MAGIC
        except OSError:
            return False
    
    def parse_stl_ascii(self, content: str) -> Tuple[float, float, float, float, float, float]:
        """Parse ASCII STL file and extract bounding box coordinates."""
        vertices = []
        for line in content.split('\n'):
            line = line.strip()
            if line.startswith('vertex'):
                parts = line.split()
                if len(parts) == 4:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        vertices.append((x, y, z))
                    except ValueError:
                        continue
        
        if not vertices:
            raise ValueError("No vertices found in STL file")
        
        xs, ys, zs = zip(*vertices)
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
    
    def parse_stl_binary(self, data: bytes) -> Tuple[float, float, float, float, float, float]:
        """Parse binary STL file and extract bounding box coordinates."""
        if len(data) < 84:
            raise ValueError("Binary STL file too small")
        
        # Skip 80-byte header and 4-byte triangle count
        num_triangles = struct.unpack('<I', data[80:84])[0]
        
        vertices = []
        offset = 84
        
        for _ in range(num_triangles):
            # Skip normal vector (12 bytes)
            offset += 12
            
            # Read 3 vertices (each 3 floats = 12 bytes)
            for _ in range(3):
                if offset + 12 <= len(data):
                    x, y, z = struct.unpack('<fff', data[offset:offset+12])
                    vertices.append((x, y, z))
                    offset += 12
            
            # Skip attribute byte count (2 bytes)
            offset += 2
        
        if not vertices:
            raise ValueError("No vertices found in binary STL file")
        
        xs, ys, zs = zip(*vertices)
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
    
    def get_stl_dimensions(self) -> Tuple[float, float, float, float, float, float]:
        """Read STL file and extract bounding box."""
        try:
            # Try ASCII format first
            content = self.input_path.read_text(encoding='utf-8')
            if 'solid' in content.lower():
                return self.parse_stl_ascii(content)
        except (UnicodeDecodeError, ValueError):
            pass
        
        # Fall back to binary format
        data = self.input_path.read_bytes()
        return self.parse_stl_binary(data)
    
    def read_gcode_text(self) -> str:
        """Return the gcode as plain text, decoding bgcode if needed."""
        if self.is_bgcode():
            return decode_bgcode_to_text(self.input_path.read_bytes())
        return self.input_path.read_text(encoding='utf-8', errors='ignore')

    def parse_gcode_dimensions(self) -> Tuple[float, float, float, float, float, float]:
        """Parse gcode file and extract max X, Y, Z coordinates."""
        content = self.read_gcode_text()

        x_coords = []
        y_coords = []
        z_coords = []
        
        # Pattern to match G0/G1 commands with coordinates
        # Matches lines like: G1 X10.5 Y20.3 Z5.1 F1200
        pattern = r'G[01]\s+(?:.*?([XY])([\d.-]+))?.*?(?:([XY])([\d.-]+))?.*?(?:(Z)([\d.-]+))?'
        
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            
            # Extract X coordinate
            x_match = re.search(r'X([-\d.]+)', line, re.IGNORECASE)
            if x_match:
                x_coords.append(float(x_match.group(1)))
            
            # Extract Y coordinate
            y_match = re.search(r'Y([-\d.]+)', line, re.IGNORECASE)
            if y_match:
                y_coords.append(float(y_match.group(1)))
            
            # Extract Z coordinate
            z_match = re.search(r'Z([-\d.]+)', line, re.IGNORECASE)
            if z_match:
                z_coords.append(float(z_match.group(1)))
        
        if not (x_coords or y_coords or z_coords):
            raise ValueError("No movement commands found in gcode file")
        
        min_x = min(x_coords) if x_coords else 0
        max_x = max(x_coords) if x_coords else 0
        min_y = min(y_coords) if y_coords else 0
        max_y = max(y_coords) if y_coords else 0
        min_z = min(z_coords) if z_coords else 0
        max_z = max(z_coords) if z_coords else 0
        
        return min_x, max_x, min_y, max_y, min_z, max_z
    
    def get_dimensions(self) -> Tuple[float, float, float]:
        """Get XYZ dimensions of the model, checking against build area."""
        if self.file_ext == '.stl':
            min_x, max_x, min_y, max_y, min_z, max_z = self.get_stl_dimensions()
        elif self.file_ext in {'.gcode', '.bgcode', '.gx', '.g'}:
            min_x, max_x, min_y, max_y, min_z, max_z = self.parse_gcode_dimensions()
        else:
            raise ValueError(f"Cannot extract dimensions from {self.file_ext}")
        
        # Calculate actual dimensions
        dim_x = max_x - min_x
        dim_y = max_y - min_y
        dim_z = max_z - min_z
        
        return dim_x, dim_y, dim_z
    
    def check_dimensions(self) -> bool:
        """Verify dimensions fit within build area."""
        dim_x, dim_y, dim_z = self.get_dimensions()
        max_x, max_y, max_z = self.BUILD_AREA
        
        print(f"Model Dimensions: {dim_x:.2f}mm × {dim_y:.2f}mm × {dim_z:.2f}mm")
        print(f"Build Area:       {max_x:.2f}mm × {max_y:.2f}mm × {max_z:.2f}mm")
        
        if dim_x > max_x or dim_y > max_y or dim_z > max_z:
            print("❌ ERROR: Model exceeds build area!")
            if dim_x > max_x:
                print(f"   X dimension {dim_x:.2f}mm exceeds max {max_x}mm")
            if dim_y > max_y:
                print(f"   Y dimension {dim_y:.2f}mm exceeds max {max_y}mm")
            if dim_z > max_z:
                print(f"   Z dimension {dim_z:.2f}mm exceeds max {max_z}mm")
            return False
        
        print("✓ Dimensions OK")
        return True
    
    def stl_to_gcode(self) -> Path:
        """Convert STL to gcode using external tool.
        
        Requires either:
        - FlashPrint CLI (https://www.flashforge.com/support)
        - Or Cura (https://ultimaker.com/software/ultimaker-cura)
        """
        output_path = self.input_path.with_suffix('.gcode')
        
        print(f"\nSTL file detected. Conversion to gcode required.")
        print(f"This script can use FlashPrint CLI or Cura for slicing.")
        print(f"\nOption 1: Use FlashPrint 5 (GUI)")
        print(f"  - Open FlashPrint 5")
        print(f"  - Import: {self.input_path}")
        print(f"  - Select Monoprice Voxel profile")
        print(f"  - Export as gcode")
        print(f"\nOption 2: Install Cura or FlashPrint CLI")
        print(f"  - FlashPrint: https://www.flashforge.com/support")
        print(f"  - Cura (with Monoprice Voxel profile): https://ultimaker.com/software/ultimaker-cura")
        
        # Try FlashPrint CLI if available
        try:
            subprocess.run(
                ['FlashPrint', '-s', str(self.input_path), '-o', str(output_path)],
                check=True,
                capture_output=True
            )
            print(f"✓ Converted to: {output_path}")
            return output_path
        except FileNotFoundError:
            pass
        except subprocess.CalledProcessError as e:
            print(f"FlashPrint conversion failed: {e.stderr.decode()}")
        
        raise RuntimeError(
            f"STL file requires conversion to gcode. No slicing tool found.\n"
            f"Please use FlashPrint 5 or Cura to convert {self.input_path} to gcode format."
        )
    
    def gcode_to_gx(self, gcode_path: Path) -> Path:
        """Convert gcode to gx format by adding Monoprice Voxel header."""
        output_path = gcode_path.with_suffix('.gx')
        
        gcode_content = gcode_path.read_bytes()
        
        # Monoprice Voxel GX format: 4-byte header + gcode content
        # Standard header for Voxel GX files
        gx_content = self.GX_HEADER + gcode_content
        
        output_path.write_bytes(gx_content)
        print(f"✓ Converted to: {output_path}")
        return output_path

    def bgcode_to_gcode(self) -> Path:
        """Decode binary gcode (bgcode) to a plain-text .gcode sibling file."""
        text = decode_bgcode_to_text(self.input_path.read_bytes())

        output_path = self.input_path.with_suffix('.gcode')
        if output_path == self.input_path:
            # Input already carries a .gcode extension but holds binary gcode.
            output_path = self.input_path.with_name(self.input_path.stem + '.decoded.gcode')

        output_path.write_text(text, encoding='utf-8')
        print(f"✓ Decoded to: {output_path}")
        return output_path

    def process(self) -> Optional[Path]:
        """Main processing pipeline."""
        print(f"Processing: {self.input_path}")
        print(f"File type: {self.file_ext}\n")
        
        # Validate extension
        self.validate_extension()
        
        # Check dimensions
        if not self.check_dimensions():
            return None
        
        # Process based on file type
        if self.file_ext == '.stl':
            print("\n→ Converting STL to gcode...")
            gcode_path = self.stl_to_gcode()
            print("\n→ Converting gcode to gx...")
            return self.gcode_to_gx(gcode_path)

        elif self.file_ext == '.bgcode' or self.is_bgcode():
            print("\n→ Decoding binary gcode (bgcode)...")
            gcode_path = self.bgcode_to_gcode()
            print("\n→ Converting gcode to gx...")
            return self.gcode_to_gx(gcode_path)

        elif self.file_ext == '.gcode':
            print("\n→ Converting gcode to gx...")
            return self.gcode_to_gx(self.input_path)

        elif self.file_ext == '.gx':
            print("\n✓ Already in GX format - no conversion needed")
            return self.input_path
        
        elif self.file_ext == '.g':
            print("\n✓ G file detected - leaving as-is for FlashPrint")
            return self.input_path
        
        return None


def main():
    if len(sys.argv) != 2:
        print("Usage: python voxel_converter.py <input_file>")
        print("Supported formats: .stl, .gcode, .bgcode, .gx, .g")
        sys.exit(1)
    
    try:
        converter = VoxelConverter(sys.argv[1])
        output = converter.process()
        
        if output:
            print(f"\n✓ Ready to print: {output}")
            sys.exit(0)
        else:
            print("\n✗ Processing failed - file exceeds build area")
            sys.exit(1)
    
    except Exception as e:
        print(f"\n✗ Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
