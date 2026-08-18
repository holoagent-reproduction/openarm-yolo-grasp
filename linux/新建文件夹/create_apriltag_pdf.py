from pathlib import Path
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

root = Path(__file__).parent
output = root / "output" / "pdf" / "AprilTag_36h11_ID0_50mm.pdf"
output.parent.mkdir(parents=True, exist_ok=True)
tag = root / "apriltag36h11_id0.png"

page_w, page_h = A4
tag_size = 50 * mm
x = (page_w - tag_size) / 2
y = (page_h - tag_size) / 2 + 8 * mm
c = canvas.Canvas(str(output), pagesize=A4)
c.setTitle("AprilTag 36h11 - ID 0")
c.setFont("Helvetica-Bold", 18)
c.drawCentredString(page_w / 2, page_h - 25 * mm, "AprilTag 36h11 - ID 0")
c.setFont("Helvetica", 10)
c.drawCentredString(page_w / 2, page_h - 32 * mm, "Hand-eye calibration marker")
c.drawImage(str(tag), x, y, width=tag_size, height=tag_size, mask="auto")
c.setLineWidth(0.5)
c.line(x, y - 8 * mm, x + tag_size, y - 8 * mm)
c.line(x, y - 10 * mm, x, y - 6 * mm)
c.line(x + tag_size, y - 10 * mm, x + tag_size, y - 6 * mm)
c.setFont("Helvetica", 10)
c.drawCentredString(page_w / 2, y - 14 * mm, "Black tag edge: 50 mm")
c.setFont("Helvetica", 8)
c.drawCentredString(page_w / 2, 22 * mm, "Print at 100% scale. Disable Fit to Page / Scale to Fit.")
c.drawCentredString(page_w / 2, 17 * mm, "Verify the black square measures exactly 50 mm before calibration.")
c.showPage(); c.save()
print(output)
