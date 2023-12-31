"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class lcm_float32Array(object):
    __slots__ = ["num_data", "data"]

    __typenames__ = ["int32_t", "float"]

    __dimensions__ = [None, ["num_data"]]

    def __init__(self):
        self.num_data = 0
        self.data = []

    def encode(self):
        buf = BytesIO()
        buf.write(lcm_float32Array._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_data))
        buf.write(struct.pack('>%df' % self.num_data, *self.data[:self.num_data]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lcm_float32Array._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcm_float32Array._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcm_float32Array()
        self.num_data = struct.unpack(">i", buf.read(4))[0]
        self.data = struct.unpack('>%df' % self.num_data, buf.read(self.num_data * 4))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if lcm_float32Array in parents: return 0
        tmphash = (0x96d521430ed4a718) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcm_float32Array._packed_fingerprint is None:
            lcm_float32Array._packed_fingerprint = struct.pack(">Q", lcm_float32Array._get_hash_recursive([]))
        return lcm_float32Array._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", lcm_float32Array._get_packed_fingerprint())[0]

