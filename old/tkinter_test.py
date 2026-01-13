import　tkinter as tk

# ウィンドウを作成
window = tk.Tk()
window.title("簡単なtkinterアプリ")
window.geometry("300x200")

# ボタンを作成
button = tk.Button(window, text="クリックしてください", command=lambda: print("ボタンがクリックされました！"))
button.pack(pady=50)

# ウィンドウを表示
window.mainloop()