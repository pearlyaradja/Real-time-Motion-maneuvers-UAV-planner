import os

# Nama file sesuai dengan yang Anda sebutkan
target_file = "PPT Sempro-MASIH PERLU DI UPDATE hanif sdv adc and scc.pptx"

def check_upload():
    if os.path.exists(target_file):
        size = os.path.getsize(target_file)
        print(f"✅ Berhasil! File '{target_file}' ditemukan.")
        print(f"📊 Ukuran file: {size / 1024:.2f} KB")
    else:
        print(f"❌ File '{target_file}' belum ditemukan di folder ini.")
        print("💡 Tips: Pastikan Anda melakukan drag-and-drop file ke folder yang aktif.")

if __name__ == "__main__":
    check_upload()