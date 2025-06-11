def get_byte_size(s):
    """
    Tính kích thước thực tế của chuỗi theo bytes
    """
    return len(s.encode('utf-8'))

def hex_to_string(hex_str):
    """
    Chuyển đổi chuỗi hex thành string
    Ví dụ: "48656C6C6F" -> "Hello"
    """
    try:
        # Loại bỏ tiền tố '0x' và khoảng trắng
        hex_str = hex_str.replace('0x', '').replace(' ', '')
        
        # Chuyển đổi hex sang bytes
        bytes_data = bytes.fromhex(hex_str)
        
        # Lọc bỏ các byte null và không hợp lệ
        filtered_bytes = bytes([b for b in bytes_data if b != 0 and 32 <= b <= 126])
        
        # Decode thành string
        result = filtered_bytes.decode('ascii')
        byte_size = len(filtered_bytes)
        
        print(f"Chuỗi gốc: {result}")
        print(f"Kích thước (bytes): {byte_size}")
        
        return result, byte_size
    except Exception as e:
        print(f"Lỗi chuyển đổi: {str(e)}")
        return "", 0

def string_to_hex(text):
    """
    Chuyển đổi string thành chuỗi hex
    Ví dụ: "Hello" -> "48656C6C6F"
    """
    try:
        # Encode string thành bytes rồi chuyển sang hex
        result = text.encode('utf-8').hex().upper()
        byte_size = get_byte_size(text)
        return result, byte_size
    except Exception as e:
        return f"Lỗi chuyển đổi: {str(e)}", 0

# Chương trình chính để test
def main():
    while True:
        print("\n=== CHƯƠNG TRÌNH CHUYỂN ĐỔI HEX <-> STRING ===")
        print("1. Chuyển HEX sang STRING")
        print("2. Chuyển STRING sang HEX")
        print("3. Tính khối lượng byte của STRING")
        print("4. Thoát")
        
        choice = input("Chọn chức năng (1-4): ")
        
        if choice == '1':
            hex_input = input("Nhập chuỗi HEX (ví dụ: 48656C6C6F): ")
            result, byte_size = hex_to_string(hex_input)
            print(f"Kết quả: {result}")
            print(f"Khối lượng byte: {byte_size} bytes")
            
        elif choice == '2':
            string_input = input("Nhập chuỗi cần chuyển sang HEX: ")
            result, byte_size = string_to_hex(string_input)
            print(f"Kết quả: {result}")
            print(f"Khối lượng byte: {byte_size} bytes")
            
        elif choice == '3':
            string_input = input("Nhập chuỗi cần tính khối lượng byte: ")
            byte_size = get_byte_size(string_input)
            print(f"Khối lượng byte: {byte_size} bytes")
            
        elif choice == '4':
            print("Tạm biệt!")
            break
            
        else:
            print("Lựa chọn không hợp lệ!")

if __name__ == "__main__":
    main()