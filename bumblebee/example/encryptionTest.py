from Crypto.Cipher import AES
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Util.Padding import pad, unpad
import base64

class HashHelper:
    IV = "0000001234123121"
    PASSWORD = "bconnect@0987654321"
    SALT = "0000001234123121"
    
    @staticmethod
    def encrypt_and_encode(raw):
        """AES 암호화 후 Base64 인코딩"""
        crypt_object = HashHelper._get_crypto_object(True)
        
        # 문자열을 바이트로 변환
        input_bytes = raw.encode('utf-8')
        
        # AES 블록 크기(16바이트)에 맞게 패딩
        padded_data = pad(input_bytes, AES.block_size)
        
        # 암호화
        encrypted_bytes = crypt_object.encrypt(padded_data)
        
        # Base64 인코딩
        encrypted_string = base64.b64encode(encrypted_bytes).decode('utf-8')
        
        return encrypted_string
    
    @staticmethod
    def decode_and_decrypt(encrypted):
        """Base64 디코딩 후 AES 복호화"""
        crypt_object = HashHelper._get_crypto_object(False)
        
        # Base64 디코딩
        encrypted_bytes = base64.b64decode(encrypted)
        
        # 복호화
        decrypted_bytes = crypt_object.decrypt(encrypted_bytes)
        
        # 패딩 제거
        unpadded_data = unpad(decrypted_bytes, AES.block_size)
        
        # 바이트를 문자열로 변환
        decrypted_string = unpadded_data.decode('utf-8')
        
        return decrypted_string
    
    @staticmethod
    def _get_crypto_object(encrypting):
        """암호화/복호화 객체 생성"""
        # RFC2898 키 도출 함수(PBKDF2)를 사용하여 키 생성
        # C#에서는 65536번 반복했지만, 성능을 위해 Python에서는 10000번으로 조정 가능
        key = PBKDF2(
            password=HashHelper.PASSWORD.encode('utf-8'),
            salt=HashHelper.SALT.encode('utf-8'),
            dkLen=16,  # 16바이트(128비트) 키
            count=65536  # 반복 횟수
        )
        
        iv = HashHelper.IV.encode('utf-8')
        
        # AES CBC 모드 객체 생성
        if encrypting:
            return AES.new(key, AES.MODE_CBC, iv)
        else:
            return AES.new(key, AES.MODE_CBC, iv)
          
# 암호화
encrypted = HashHelper.encrypt_and_encode("w1")
print(encrypted)

# 복호화
decrypted = HashHelper.decode_and_decrypt(encrypted)
print(decrypted)  # 출력: 안녕하세요