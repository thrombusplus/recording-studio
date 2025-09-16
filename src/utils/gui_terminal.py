import logging
import queue
import datetime
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText
from typing import Optional
from src.utils.logger import AsciiFormatter


# Custom logging handler
class _GUITextHandler(logging.Handler):
    def __init__(self, log_queue: queue.Queue):
        super().__init__()
        self._q = log_queue

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            lvl = record.levelno
        except Exception:
            msg = f"Logging formatting error:\n{traceback.format_exc()}"
            lvl = logging.ERROR
        try:
            self._q.put((lvl, msg))
        except Exception:
            pass

class LogTab(ttk.Frame):
    def __init__(self, parent, *, poll_ms: int = 60, font=("Consolas", 12), max_lines: int = 1500):
        super().__init__(parent)
        self._poll_ms = int(poll_ms)
        self._q = queue.Queue()
        self._handler = _GUITextHandler(self._q)

        self._handler.setFormatter(AsciiFormatter)

        self._after_id: Optional[str] = None
        self._running = True

        # filter / sizing
        self._max_lines = int(max_lines)

        # Toolbar
        bar = ttk.Frame(self)
        bar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=6)

        ttk.Button(bar, text="Clear", command=self.clear).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Save…", command=self.save_to_file).pack(side=tk.LEFT, padx=4)

        # Text area
        self.text = ScrolledText(self, wrap="word", state="disabled")
        self.text.configure(font=font)
        self.text.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

        # coloring
        self.text.tag_configure("DEBUG", foreground="green")
        self.text.tag_configure("INFO", foreground="black")
        self.text.tag_configure("WARNING", foreground="orange")
        self.text.tag_configure("ERROR", foreground="red")

        # tracking attached logger & original level
        self._attached_logger: Optional[logging.Logger] = None
        self._attached_logger_original_level: Optional[int] = None

        try:
            self._after_id = self.after(self._poll_ms, self._poll_queue)
        except tk.TclError:
            self._after_id = None

    # Public API
    def get_handler(self) -> logging.Handler:
        return self._handler

    def attach(self, logger_name: Optional[str] = None, level: int = logging.DEBUG):
        target = logging.getLogger(logger_name) if logger_name else logging.getLogger()

    # detach
        if self._attached_logger is not None and self._attached_logger is not target:
            self.detach()

        if self._handler not in target.handlers:
            target.addHandler(self._handler)

            try:
                self._attached_logger_original_level = target.level
            except Exception:
                self._attached_logger_original_level = None

            try:
                target.setLevel(level)
            except Exception:
                pass

            self._attached_logger = target

    def detach(self):
        if self._attached_logger is not None:
            try:
                if self._handler in self._attached_logger.handlers:
                    self._attached_logger.removeHandler(self._handler)
            except Exception:
                pass

            try:
                if self._attached_logger_original_level is not None:
                    self._attached_logger.setLevel(self._attached_logger_original_level)
            except Exception:
                pass

        self._attached_logger = None
        self._attached_logger_original_level = None

    def _poll_queue(self):
        if not self.winfo_exists() or not getattr(self, "_running", True):
            return
        try:
            while True:
                lvl, msg = self._q.get_nowait()
                level_name = logging.getLevelName(lvl)
                self._append_line(msg, tag=level_name)
        except queue.Empty:
            pass

        try:
            self._after_id = self.after(self._poll_ms, self._poll_queue)
        except tk.TclError:
            self._after_id = None
            return

    def _append_line(self, line: str, tag: Optional[str] = None):
        try:
            self.text.configure(state="normal")
            if tag:
                self.text.insert("end", line + "\n", tag)
            else:
                self.text.insert("end", line + "\n")
                self.text.configure(state="disabled")

            # Scroll to the end of the text
            self.text.update_idletasks()
            self.text.see("end")

            self.trim_lines()
        except tk.TclError:
            pass

    def trim_lines(self):
        try:
            doc = self.text
            lines = int(doc.index("end-1c").split(".")[0])
            if lines > self._max_lines:
                to_remove = max(int(self._max_lines * 0.1), 1)
                delete_end = f"{to_remove + 1}.0"
                doc.configure(state="normal")
                doc.delete("1.0", delete_end)
                doc.configure(state="disabled")
        except tk.TclError:
            pass

    def clear(self):
        try:
            self.text.configure(state="normal")
            self.text.delete("1.0", "end")
            self.text.configure(state="disabled")
        except tk.TclError:
            pass

    def save_to_file(self):
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
            defaultextension=".log",
            initialfile=f"app_{now}.log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            data = self.text.get("1.0", "end")
            with open(path, "w", encoding="utf-8") as f:
                f.write(data)
            messagebox.showinfo("Saved", f"Logs saved to:\n{path}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    def destroy(self):
        self._running = False
        try:
            if self._after_id is not None:
                self.after_cancel(self._after_id)
                self._after_id = None
        except Exception:
            pass

        try:
            self.detach()
        except Exception:
            pass

        try:
            super().destroy()
        except Exception:
            pass
                