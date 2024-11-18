import os
import json
from pathlib import Path
from reportlab.platypus import (
    BaseDocTemplate,
    PageTemplate,
    Frame,
    Image,
    Paragraph,
    Spacer,
)
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import inch, mm
from reportlab.pdfgen import canvas
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_JUSTIFY
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

# Register Times New Roman font (optional)
# Uncomment and adjust the path to the font file if available
# pdfmetrics.registerFont(TTFont('Times-Roman', 'Times.ttf'))


class CartographerDocTemplate(BaseDocTemplate):
    def __init__(self, filename, **kwargs):
        # Set custom margins
        self.leftMargin = 10 * mm
        self.rightMargin = 10 * mm
        self.topMargin = 15 * mm
        self.bottomMargin = 15 * mm

        super().__init__(filename, **kwargs)
        page_width, page_height = self.pagesize

        # Margins
        margin_left = self.leftMargin
        margin_right = self.rightMargin
        margin_top = self.topMargin
        margin_bottom = self.bottomMargin

        # Column settings
        num_columns = 2
        column_gap = 4 * mm  # Reduced gap between columns
        column_width = (
            page_width - margin_left - margin_right - (num_columns - 1) * column_gap
        ) / num_columns
        column_height = page_height - margin_top - margin_bottom

        # Define frames for the columns
        frames = []
        for i in range(num_columns):
            frame = Frame(
                margin_left + i * (column_width + column_gap),
                margin_bottom,
                column_width,
                column_height,
                leftPadding=0,
                rightPadding=0,
                topPadding=0,
                bottomPadding=0,
                id=f"col{i}",
            )
            frames.append(frame)

        # Create a PageTemplate with the frames
        template = PageTemplate(
            id="TwoColumns", frames=frames, onPage=self.add_page_number_footer
        )
        self.addPageTemplates([template])

    def add_page_number_footer(self, canvas, doc):
        canvas.saveState()
        canvas.setFont("Times-Roman", 9)

        # Add footer text
        footer_text = "MeshCartographer - Report"
        canvas.drawString(self.leftMargin, 12 * mm, footer_text)

        # Add page number
        page_number_text = f"Page {canvas.getPageNumber()}"
        canvas.drawRightString(
            doc.pagesize[0] - self.rightMargin, 12 * mm, page_number_text
        )

        # Optional: Add a logo at the top right corner
        image_path = Path(__file__).resolve().parent / "assets" / "logo.png"

        if image_path.exists():
            image_width = 20 * mm
            image_height = image_width
            canvas.drawImage(
                image_path,
                doc.pagesize[0] - self.rightMargin - image_width,
                doc.pagesize[1] - self.topMargin - image_height,
                width=image_width,
                height=image_height,
                preserveAspectRatio=True,
                mask="auto",
            )

        canvas.restoreState()


def create_custom_pdf(image_path, pdf_path, data):
    try:
        # Create an instance of the CartographerDocTemplate
        doc = CartographerDocTemplate(pdf_path, pagesize=A4)

        # Create a style sheet and customize styles
        styles = getSampleStyleSheet()

        # Define custom styles
        styles.add(
            ParagraphStyle(
                name="EconomistTitle",
                parent=styles["Title"],
                fontName="Times-Roman",
                fontSize=18,
                leading=22,
                alignment=TA_JUSTIFY,
                spaceAfter=6,
            )
        )

        styles.add(
            ParagraphStyle(
                name="EconomistBody",
                parent=styles["Normal"],
                fontName="Times-Roman",
                fontSize=10,
                leading=13,
                alignment=TA_JUSTIFY,
                spaceAfter=6,
            )
        )

        # Calculate the column width
        page_width, _ = doc.pagesize
        margin_left = doc.leftMargin
        margin_right = doc.rightMargin
        column_gap = 4 * mm
        num_columns = 2
        column_width = (
            page_width - margin_left - margin_right - (num_columns - 1) * column_gap
        ) / num_columns

        # Add elements to the flowable elements
        elements = []

        # Title
        title = Paragraph("Mesh Cartographer Report", styles["EconomistTitle"])
        elements.append(title)
        elements.append(Spacer(1, 5 * mm))

        # Initial area
        initial_area = data.get("initial_area", "N/A")
        initial_area_paragraph = Paragraph(
            f"Initial Area: {initial_area}", styles["EconomistBody"]
        )
        elements.append(initial_area_paragraph)
        elements.append(Spacer(1, 5 * mm))

        # Add the image, scaled to fit the column width
        img = Image(image_path)
        img.drawWidth = column_width
        img.drawHeight = img.drawWidth * img.imageHeight / img.imageWidth
        elements.append(img)
        elements.append(Spacer(1, 5 * mm))

        # Build the PDF
        doc.build(elements)
        print(f"Custom PDF saved at: {pdf_path}")
    except Exception as e:
        print(f"An error occurred while creating the custom PDF: {e}")


def main():
    # Path to the image generated by generate_data.py
    image_filename = "img/spectre_border.png"
    image_path = Path(image_filename)

    if not image_path.exists():
        print(
            f"Image file {image_filename} does not exist. Please run generate_data.py first."
        )
        return

    # Load data from data.json
    data_filename = "data.json"
    if not os.path.exists(data_filename):
        print(
            f"Data file {data_filename} does not exist. Please run generate_data.py first."
        )
        return
    with open(data_filename, "r") as f:
        data = json.load(f)

    # Create the 'pdf' directory if it doesn't exist
    pdf_folder = "pdf"
    if not os.path.exists(pdf_folder):
        os.makedirs(pdf_folder)

    # Define the path for the PDF file
    pdf_filename = "mesh_cartographer.pdf"
    pdf_path = os.path.join(pdf_folder, pdf_filename)

    # Call the function to create the PDF
    create_custom_pdf(str(image_path), pdf_path, data)


if __name__ == "__main__":
    main()
