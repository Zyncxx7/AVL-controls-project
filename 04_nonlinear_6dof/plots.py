"""Publication plots using ReportLab numeric line charts; no Matplotlib needed."""
from pathlib import Path
import shutil
import subprocess
from math import ceil, floor, log10

from reportlab.graphics.shapes import Drawing, String, Line, Group
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics.widgets.markers import makeMarker
from reportlab.graphics import renderPDF
from reportlab.lib import colors
from reportlab.pdfgen.canvas import Canvas

RESULTS=Path(__file__).resolve().parent/'results'
PALETTE=[colors.HexColor(c) for c in ('#1663A6','#21865B','#9C3C99','#D06B16','#B52A3C')]


def tick_step(span, intervals):
    rough=span/intervals
    base=10**floor(log10(rough))
    return min((1,2,2.5,5,10),key=lambda v:abs(v-rough/base))*base


def panel(drawing,series,x_label,y_label,xmin,xmax,ymin,ymax,x=90,y=105,width=630,height=360):
    plot=LinePlot()
    plot.x,plot.y,plot.width,plot.height=x,y,width,height
    plot.data=[points for _,points,_ in series]
    plot.joinedLines=1
    plot.xValueAxis.valueMin,plot.xValueAxis.valueMax=xmin,xmax
    plot.yValueAxis.valueMin,plot.yValueAxis.valueMax=ymin,ymax
    plot.xValueAxis.valueStep=tick_step(xmax-xmin,8)
    plot.yValueAxis.valueStep=tick_step(ymax-ymin,5)
    for axis in (plot.xValueAxis,plot.yValueAxis):
        axis.labels.fontName='Helvetica'
        axis.labels.fontSize=9
        axis.labelTextFormat=lambda value:f'{value:.4g}'
        axis.visibleGrid=1
        axis.gridStrokeColor=colors.HexColor('#DDE1E5')
        axis.gridStrokeWidth=0.5
    for i,(_,points,color) in enumerate(series):
        plot.lines[i].strokeColor=color
        plot.lines[i].strokeWidth=1.5
        plot.lines[i].symbol=makeMarker('Circle',size=3,fillColor=color,strokeColor=color) if len(points)==1 else None
    drawing.add(plot)
    drawing.add(String(x+width/2,y-37,x_label,textAnchor='middle',fontName='Helvetica',fontSize=11))
    label=Group(String(0,0,y_label,textAnchor='middle',fontName='Helvetica',fontSize=11))
    label.transform=(0,1,-1,0,30,y+height/2)
    drawing.add(label)


def heading(drawing,title,setup):
    drawing.add(String(45,570,title,fontName='Helvetica-Bold',fontSize=19))
    for i,line in enumerate(setup):
        drawing.add(String(45,545-i*15,line,fontName='Helvetica',fontSize=10))


def legend(drawing,items,y=485):
    for i,(label,color) in enumerate(items):
        col=i%3
        row=i//3
        x=80+col*230
        yy=y-row*18
        drawing.add(Line(x,yy,x+20,yy,strokeColor=color,strokeWidth=2))
        drawing.add(String(x+26,yy-3,label,fontName='Helvetica',fontSize=9))


def split_series(rows,key):
    """Avoid drawing across speeds with missing or infeasible trim points."""
    segments=[]
    segment=[]
    for row in sorted(rows,key=lambda r:r['airspeed_kt']):
        if segment and row['airspeed_kt']-segment[-1][0]>1.01:
            segments.append(segment)
            segment=[]
        segment.append((row['airspeed_kt'],row[key]))
    if segment: segments.append(segment)
    return segments


def trim_series(trims,key):
    series=[]
    categories=[('Original DATCOM',False),('Original AVL',False),('PDF attached',False),('PDF stalled',False),('PDF attached',True)]
    items=[]
    for i,(name,post) in enumerate(categories):
        rows=[r for r in trims if r['model']==name and r['feasible']
              and (name!='PDF attached' or (r.get('root_region')=='post-peak')==post)]
        label='PDF attached, post-peak' if post else name
        for segment in split_series(rows,key):
            series.append((label,segment,PALETTE[i]))
        if rows: items.append((label,PALETTE[i]))
    return series,items


def make_plots(trims,cycle,responses):
    pages=[]
    d=Drawing(792,612)
    heading(d,'Trim elevator versus airspeed',[
        'Straight level flight; sea-level density; clean wing; mass 1043 kg; common propeller model.',
        'Airspeed 40-125 kt in 1 kt steps; zero body rates; only roots with throttle between 0 and 1 are plotted.'])
    series,items=trim_series(trims,'elevator_deg')
    legend(d,items)
    ymin=min(p[1] for _,points,_ in series for p in points)-1
    ymax=max(p[1] for _,points,_ in series for p in points)+1
    panel(d,series,'True airspeed (kt)','Trim elevator (deg)',40,125,floor(ymin/5)*5,ceil(ymax/5)*5,height=325)
    pages.append(('trim_elevator',d))
    d=Drawing(792,612)
    heading(d,'Trim angle of attack and throttle',[
        'Same level-flight cases and constraints as the elevator comparison.'])
    series,items=trim_series(trims,'alpha_deg')
    legend(d,items,y=510)
    panel(d,series,'True airspeed (kt)','Trim alpha (deg)',40,125,-5,25,y=310,height=150)
    series,_=trim_series(trims,'throttle')
    panel(d,series,'True airspeed (kt)','Trim throttle',40,125,0,1,y=85,height=150)
    pages.append(('trim_parameters',d))
    d=Drawing(792,612)
    heading(d,'Increasing and decreasing alpha: stall memory',[
        'Prescribed alpha cycle: -0.05 to 0.361 rad and back; sampling interval 0.02 s.',
        'Hysteresis sets above 0.36 rad and resets below 0.09 rad; wing-body CL is plotted.'])
    series=[(direction,[(r['alpha_deg'],r['CL_wing']) for r in cycle if r['direction']==direction],PALETTE[i])
            for i,direction in enumerate(('increasing','decreasing'))]
    legend(d,[(s[0],s[2]) for s in series],y=492)
    panel(d,series,'Angle of attack (deg)','Wing-body lift coefficient',-5,22,-0.25,1.6,y=250,height=205)
    series=[('memory',[(r['time_s'],r['hysteresis']) for r in cycle],PALETTE[2])]
    panel(d,series,'Time in prescribed cycle (s)','Hysteresis state',0,cycle[-1]['time_s'],-0.1,1.1,y=85,height=90)
    pages.append(('stall_loop',d))
    d=Drawing(792,612)
    heading(d,'6DOF time response to elevator input',[
        'Each model starts at its own 100 kt level-flight trim; sea-level density; duration 15 s; timestep 0.01 s.',
        'Additional elevator -1 deg from 2 to 3 s; trim throttle held fixed; clean configuration.'])
    items=[('Original DATCOM',PALETTE[0]),('Original AVL',PALETTE[1]),('PDF nonlinear',PALETTE[2])]
    legend(d,items,y=492)
    for j,(key,label) in enumerate((('speed_kt','Airspeed (kt)'),('pitch_deg','Pitch (deg)'),('altitude_m','Altitude (m)'))):
        series=[(name,[(r['time_s'],r[key]) for r in responses if r['model']==name],PALETTE[i]) for i,name in enumerate(('DATCOM','AVL','PDF'))]
        values=[p[1] for _,points,_ in series for p in points]
        span=max(values)-min(values)
        margin=max(0.1,span*0.15)
        panel(d,series,'Time (s)',label,0,15,min(values)-margin,max(values)+margin,y=350-j*135,height=95)
    pages.append(('time_response',d))
    output=RESULTS/'research_results.pdf'
    canvas=Canvas(str(output),pagesize=(792,612))
    canvas.setTitle('C172 trim comparison and nonlinear stall hysteresis')
    for _,drawing in pages:
        renderPDF.draw(drawing,canvas,0,0)
        canvas.showPage()
    canvas.save()
    poppler=shutil.which('pdftoppm')
    if not poppler:
        candidate=Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/native/poppler/Library/bin/pdftoppm.exe'
        if candidate.is_file(): poppler=str(candidate)
    if poppler:
        for i,(name,_) in enumerate(pages,1):
            subprocess.run([poppler,'-f',str(i),'-l',str(i),'-singlefile','-r','150','-png',str(output),str(RESULTS/name)],check=True,capture_output=True)
    else:
        print('PDF plots saved. Install Poppler to also render PNG previews.')
