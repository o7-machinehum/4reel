Ovrdrive
--------
Size :	36.6 x 15 mm
Layer : 4 Layers
Material : FR-4: TG150
Thickness : 1.6 mm nominal (PCBWay regular 4-layer construction)
Min track/spacing : 4/4mil
Min hole size : 0.2mm
Solder mask : Black
Silkscreen : White
Edge connector : No
Surface finish : HASL lead free
Via process : Tenting vias
Finished copper : 1 oz Cu (Inner Copper:1 oz)
Remove product No. : Yes

PCBWay regular 4-layer stackup (nominal 1.6 mm)
-------------------------------------------------
Selected construction: PCBWay "4-layers PCB Regular" with 1 oz finished
outer copper and 1 oz inner copper.

Top to bottom:
L1 Cu : 0.5 oz base copper, plated to 1 oz finished (~0.035 mm)
PP    : 7628 RC46%, Dk 4.7, 0.1855 mm after lamination
L2 Cu : 1 oz (0.035 mm), continuous GND reference plane
Core  : Dk 4.6, 1.0300 mm dielectric (1.1 mm including L2/L3 copper)
L3 Cu : 1 oz (0.035 mm), continuous GND reference plane
PP    : 7628 RC46%, Dk 4.7, 0.1855 mm after lamination
L4 Cu : 0.5 oz base copper, plated to 1 oz finished (~0.035 mm)

PCBWay lists this construction at approximately 1.51 mm finished thickness,
with +/-10% tolerance, ordered as nominal 1.6 mm.

100 ohm differential Ethernet routing
--------------------------------------
Geometry: edge-coupled microstrip on L1/L4, referenced to the adjacent GND plane
Target differential impedance : 100 ohm +/-10%
Trace width                   : 0.160 mm (6.30 mil)
Pair gap, edge-to-edge         : 0.125 mm (4.92 mil)
Reference-plane height         : 0.1855 mm
Finished outer copper          : approximately 0.035 mm
Dielectric constant used       : 4.7

IPC/Wadell closed-form estimate without solder mask: approximately 101 ohm
differential. Solder mask, actual material properties, copper shape, and PCBWay
process compensation are not included in that estimate. Order the PCB with
100 ohm differential impedance control and allow PCBWay to adjust the production
width/gap after their field-solver review.
