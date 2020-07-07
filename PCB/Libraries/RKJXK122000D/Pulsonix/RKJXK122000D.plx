PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//270078/260881/2.46/14/4/Switch

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c165_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
	)
	(padStyleDef "c252_h168"
		(holeDiam 1.68)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.520) (shapeHeight 2.520))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.520) (shapeHeight 2.520))
	)
	(padStyleDef "c240_h160"
		(holeDiam 1.6)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.400) (shapeHeight 2.400))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.400) (shapeHeight 2.400))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "RKJXK122000D" (originalName "RKJXK122000D")
		(multiLayer
			(pad (padNum 1) (padStyleRef c165_h110) (pt -17.500, 9.150) (rotation 90))
			(pad (padNum 2) (padStyleRef c165_h110) (pt -17.500, -4.150) (rotation 90))
			(pad (padNum 3) (padStyleRef c165_h110) (pt 0.000, 2.500) (rotation 90))
			(pad (padNum 4) (padStyleRef c165_h110) (pt -9.750, 12.250) (rotation 90))
			(pad (padNum 5) (padStyleRef c165_h110) (pt 0.000, 5.000) (rotation 90))
			(pad (padNum 6) (padStyleRef c165_h110) (pt -7.250, 12.250) (rotation 90))
			(pad (padNum 7) (padStyleRef c252_h168) (pt -12.925, -5.750) (rotation 90))
			(pad (padNum 8) (padStyleRef c252_h168) (pt -12.925, -10.250) (rotation 90))
			(pad (padNum 9) (padStyleRef c252_h168) (pt -6.575, -5.750) (rotation 90))
			(pad (padNum 10) (padStyleRef c252_h168) (pt -6.575, -10.250) (rotation 90))
			(pad (padNum 11) (padStyleRef c240_h160) (pt -17.500, 9.150) (rotation 90))
			(pad (padNum 12) (padStyleRef c240_h160) (pt -17.500, -4.150) (rotation 90))
			(pad (padNum 13) (padStyleRef c240_h160) (pt -2.000, -4.150) (rotation 90))
			(pad (padNum 14) (padStyleRef c240_h160) (pt -2.000, 9.150) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt -8.938, 0.782) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -17.75 -5.5) (pt -1.75 -5.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.75 -5.5) (pt -1.75 10.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.75 10.5) (pt -17.75 10.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -17.75 10.5) (pt -17.75 -5.5) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -17.75 7.25) (pt -17.75 7.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -17.75 7.25) (pt -17.75 -2.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -17.75 -2.25) (pt -17.75 -2.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -17.75 -2.25) (pt -17.75 7.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.75 -2.25) (pt -1.75 -2.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.75 -2.25) (pt -1.75 6.75) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.75 6.75) (pt -1.75 6.75) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.75 6.75) (pt -1.75 -2.25) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.5 10.5) (pt -16.25 10.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -16.25 10.5) (pt -16.25 10.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -16.25 10.5) (pt -3.5 10.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.5 10.5) (pt -3.5 10.5) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -19.7 14.075) (pt 1.825 14.075) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 1.825 14.075) (pt 1.825 -12.51) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 1.825 -12.51) (pt -19.7 -12.51) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -19.7 -12.51) (pt -19.7 14.075) (width 0.1))
		)
	)
	(symbolDef "RKJXK122000D" (originalName "RKJXK122000D")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 1000 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 1000 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 1000 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 1000 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 12) (pt 1000 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 13) (pt 1000 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 14) (pt 1000 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 770 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 800 mils 100 mils) (width 6 mils))
		(line (pt 800 mils 100 mils) (pt 800 mils -700 mils) (width 6 mils))
		(line (pt 800 mils -700 mils) (pt 200 mils -700 mils) (width 6 mils))
		(line (pt 200 mils -700 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 850 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 850 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "RKJXK122000D" (originalName "RKJXK122000D") (compHeader (numPins 14) (numParts 1) (refDesPrefix S)
		)
		(compPin "11" (pinName "11") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "12" (pinName "12") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "21" (pinName "21") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "22" (pinName "22") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "31" (pinName "31") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "32" (pinName "32") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "A1" (pinName "A1") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "B1" (pinName "B1") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "C1" (pinName "C1") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "D1" (pinName "D1") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH1" (pinName "MH1") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH2" (pinName "MH2") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH3" (pinName "MH3") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH4" (pinName "MH4") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "RKJXK122000D"))
		(attachedPattern (patternNum 1) (patternName "RKJXK122000D")
			(numPads 14)
			(padPinMap
				(padNum 1) (compPinRef "11")
				(padNum 2) (compPinRef "12")
				(padNum 3) (compPinRef "21")
				(padNum 4) (compPinRef "22")
				(padNum 5) (compPinRef "31")
				(padNum 6) (compPinRef "32")
				(padNum 7) (compPinRef "A1")
				(padNum 8) (compPinRef "B1")
				(padNum 9) (compPinRef "C1")
				(padNum 10) (compPinRef "D1")
				(padNum 11) (compPinRef "MH1")
				(padNum 12) (compPinRef "MH2")
				(padNum 13) (compPinRef "MH3")
				(padNum 14) (compPinRef "MH4")
			)
		)
		(attr "Manufacturer_Name" "ALPS")
		(attr "Manufacturer_Part_Number" "RKJXK122000D")
		(attr "Mouser Part Number" "688-RKJXK122000D")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/ALPS/RKJXK122000D?qs=m0BA540hBPe588JnPdeybQ%3D%3D")
		(attr "RS Part Number" "")
		(attr "RS Price/Stock" "")
		(attr "Description" "Miniature potentiomeric joystick Joystick Switch Through Hole, -10 ??? +70degC")
		(attr "<Hyperlink>" "http://www.alps.com/products/WebObjects/catalog.woa/E/HTML/MultiControl/Potentiometer/RKJXK/RKJXK122000D.html.pdf")
		(attr "<Component Height>" "19.7")
		(attr "<STEP Filename>" "RKJXK122000D.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
