def calculate_crc8(data):
    # Khởi tạo giá trị CRC ban đầu
    crc = 0xFF
    
    # Với mỗi byte trong dữ liệu đầu vào
    for byte in data:
        # XOR byte hiện tại với giá trị CRC
        crc ^= byte
        
        # Xử lý 8 bit
        for _ in range(8):
            # Kiểm tra bit cao nhất (MSB)
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
                
        # Đảm bảo giá trị CRC nằm trong phạm vi 8 bit
        crc = crc & 0xFF
        
    return crc

def hex_string_to_bytes(hex_str):
    # Loại bỏ khoảng trắng và tách thành các chuỗi hex
    hex_values = hex_str.strip().split()
    # Chuyển đổi từng giá trị hex thành số nguyên
    return [int(x, 16) for x in hex_values]

def main():
    print("Chương trình tính CRC8 (Nhấn Ctrl+C để thoát)")
    print("----------------------------------------")
    print("Định dạng nhập: ")
    print("1. Số thập phân: 13 7 1 0")
    print("2. Số hex: 0d 07 01 00")
    
    try:
        while True:
            try:
                # Nhập chuỗi từ người dùng
                input_str = input("\nNhập giá trị (hex hoặc decimal): ")
                
                # Kiểm tra xem đầu vào có phải là hex không
                is_hex = any(x.lower().startswith('0x') or 
                           any(c.lower() in 'abcdef' for c in x) 
                           for x in input_str.split())
                
                if is_hex:
                    # Xử lý đầu vào hex
                    data = hex_string_to_bytes(input_str)
                else:
                    # Xử lý đầu vào decimal
                    data = [int(x) for x in input_str.split()]
                
                # Tính CRC8
                result = calculate_crc8(data)
                print(f"Dữ liệu (hex): {' '.join([f'{x:02X}' for x in data])}")
                print(f"Giá trị CRC8: {result} (0x{result:02X})")
                
            except ValueError:
                print("Lỗi: Vui lòng nhập đúng định dạng số!")
            except Exception as e:
                print(f"Lỗi không xác định: {str(e)}")
                
    except KeyboardInterrupt:
        print("\n\nĐã thoát chương trình.")

if __name__ == "__main__":
    main()