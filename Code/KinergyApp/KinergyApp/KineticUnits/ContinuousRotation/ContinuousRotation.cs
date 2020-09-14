using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;
using Rhino;
using Kinergy.Utilities;
using Kinergy.Geom;
using Kinergy.Relationship;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;
using HumanUIforKinergy.KinergyUtilities;

namespace Kinergy.KineticUnit
{
    class ContinuousRotation : KineticUnit
    {
        //The initial inputs
        private Brep _model;
        private double _energy; // the range on the interface is 0.1-1, the ratio of the max energy
        private double _speed; // the range on the interface is 0-9, indicating the speed level
        private Vector3d _direction = Vector3d.Unset;
        private bool _addLock;
        RhinoDoc _myDoc;
        private int _inputType;
        Brep _innerCavity;


        private Curve skeleton;
        private List<Shape> modelCut;
        private List<Lock> locks;
        private Helix springH;
        private Spiral springS;
        private Brep innerShell;
        private int outputAxelInOut = 1;        // 1: in; 2: out

        ProcessingWin processingwin = new ProcessingWin();

        double springWidthThreadshold = 20;

        public ContinuousRotation(Brep Model, Vector3d Direction, double Energy, double Speed, int InputType, Brep inCavity)
        {
            this._model = Model;
            this._energy = Energy;
            this._speed = Speed;
            this._direction = Direction;
            this.modelCut = new List<Shape>();
            this._myDoc = RhinoDoc.ActiveDoc;
            this.locks = new List<Lock>();
            this._inputType = InputType;
            this._innerCavity = inCavity;
            this.skeleton = null;

        }

        /// <summary>
        /// Generate the gear train inside the model
        /// </summary>
        /// <param name="startPoint">the position of thepinion for the input gear</param>
        /// <param name="xEnd">the center position of the output gear</param>
        /// <param name="outDiameter">the height (z) of the model that the gear train is added to</param>
        /// <param name="totalThickness">the thickness (y) of the model</param>
        /// <param name="xSpaceEnd">the length (x) of the model</param>
        /// <param name="outputAxle">the type of the output gear shaft</param>
        /// <returns></returns>
        public void ConstructGearTrain(Point3d startPoint, double xEnd, double outDiameter, double totalThickness, double xSpaceEnd, int outputAxle,
            Transform translateBack, Transform rotationBack, Transform postRotationBack)
        {

            List<Brep> result = new List<Brep>();

            #region Step 0: Create a shell

            Brep[] innerShells;
            Brep[] innerWalls;

            processingwin.Show();
            Brep[] shells = Brep.CreateOffsetBrep(Model, -1.6, false, false, MyDoc.ModelAbsoluteTolerance, out innerShells, out innerWalls);

            innerShell = shells[0];
            processingwin.Hide();

            Plane xySplitPln = new Plane(Model.GetBoundingBox(true).Center, new Vector3d(0, 0, 1));

            Point3d[] ioPts = new Point3d[2];
            Vector3d upZ = new Vector3d(0, 0, 1);
            ioPts[0] = innerShell.GetBoundingBox(true).Center - upZ * (innerShell.GetBoundingBox(true).Max.Z - innerShell.GetBoundingBox(true).Min.Z);
            ioPts[1] = innerShell.GetBoundingBox(true).Center + upZ * (innerShell.GetBoundingBox(true).Max.Z - innerShell.GetBoundingBox(true).Min.Z);
            Curve railio = new Polyline(ioPts).ToNurbsCurve();

            Brep link = new Brep();
            link = Brep.CreatePipe(railio, 1, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            Brep innerShellDup = innerShell.DuplicateBrep();
            var ioBreps = Brep.CreateBooleanUnion(new List<Brep> { innerShellDup, link }, MyDoc.ModelAbsoluteTolerance);
            if (ioBreps == null)
            {
                link.Flip();
                ioBreps = Brep.CreateBooleanUnion(new List<Brep> { innerShellDup, link }, MyDoc.ModelAbsoluteTolerance);
            }

            Brep ioBrep = ioBreps[0];

            #region test by LH
            //MyDoc.Objects.AddBrep(ioBrep);
            //MyDoc.Views.Redraw();
            #endregion

            var finalShellBreps = Brep.CreateBooleanDifference(Model, ioBrep, MyDoc.ModelAbsoluteTolerance);
            if (finalShellBreps == null)
            {
                ioBrep.Flip();
                finalShellBreps = Brep.CreateBooleanDifference(Model, ioBrep, MyDoc.ModelAbsoluteTolerance);
            }
            Brep finalShellBrep = finalShellBreps[0];

            Model = finalShellBrep.DuplicateBrep();

            #region test by LH
            //MyDoc.Objects.AddBrep(finalShellBrep);
            //MyDoc.Views.Redraw();
            #endregion

            #endregion

            #region Step 1: construct the gear train 

            if (InputType == 1)
            {
                springWidthThreadshold = 15;
            }
            else
            {
                springWidthThreadshold = 0;
            }

            Point3d gearTrainStPos = startPoint + new Vector3d(1,0,0) * springWidthThreadshold;

            double xStart = gearTrainStPos.X;
            double xLength = xEnd - xStart;
            double gap = 0.6;
            int pinionTeethNum = 10; //fixed
            double minRatio = 1.1; //to ensure that the gear teeth number is integral
            double module0 = 0.75;
            double addendum = module0;
            double pitch = Math.PI * module0;
            //gear facewidth = min(1.6, pitch*3)  
            double thickness = Math.Min(2, 3 * pitch);
            double backlash = 0.6;
            double backlashGap = 1.374;
            double maxGearDiameter = outDiameter * 0.3;
            double pairGearRatio = minRatio;
            double module = module0;
            double pressureAngle = 20;
            double tolerance = 0.6;
            double discThickness = 1;
            double shaftRadius = 1.2;
            double discRadius = 2;
            Point3d lastGearPos = new Point3d();

            if (xLength < 0) return;

            #region New version by LH

            int numPairMax = -1; // the max num of two-gear pair can be added

            //numPairMax = (int)(xLength / (backlash * backlashGap
            //                + module0 * pinionTeethNum * (minRatio + 1) / 2));


            numPairMax = (int)(xLength / (backlash * backlashGap
                            + Math.Min(module0 * pinionTeethNum * 5 / 2, maxGearDiameter/2) + module0 * pinionTeethNum / 2));
            // find the range of number of two-gear pairs that can be added in this space
            int numPairMin = 100;
            numPairMax = (int)Math.Min(numPairMax, (totalThickness - 2 * gap) / (gap + thickness));

            if(numPairMax >= 1)
            {
                
                for(int num = 1; num <= numPairMax; num++)
                {
                    double r = (xLength / num - backlash * backlashGap) * 2 / (module0 * pinionTeethNum) - 1;
                    if(r > 5)
                    {
                        numPairMin = num + 1;
                    }
                }
            }

            if (numPairMax < numPairMin) return;
            int numSection = numPairMax - numPairMin + 1;
            int currNum = (int)Math.Floor(Speed / (10.0 / numSection)) + numPairMin;

            int baseSpeed = (int)Math.Ceiling((currNum - numPairMin) * (10.0 / numSection));
            int iterationNum = (int)Speed - baseSpeed + 1;

            // start from two identical gears: R = 1

            double mgr = (xLength / currNum - backlash * backlashGap);

            int teethMax = (int)(mgr / module0);
            int teethEqualInt = (teethMax + pinionTeethNum) / 2;

            double mNew = mgr / teethEqualInt;
            double grNew = (xLength / currNum - backlash * backlashGap) * 2 / (mNew * teethEqualInt) - 1;

            for (int i = 1; i <= iterationNum; ++i)
            {
                double newMGR = mgr - (mgr - module0 * pinionTeethNum) * i / iterationNum;

                teethMax = (int)(newMGR / module0);

                teethEqualInt = (teethMax + pinionTeethNum) / 2;

                mNew = newMGR / teethEqualInt;
                grNew = (xLength / currNum - backlash * backlashGap) * 2 / (mNew * teethEqualInt) - 1;
            }

            double currModule = (xLength - currNum * backlashGap * backlash) / (currNum * teethEqualInt * (1 + grNew) / 2);
            int gearTeethNum = (int)(teethEqualInt * grNew);

            double gearDistance = teethEqualInt * (1 + grNew) * currModule / 2 + backlash * backlashGap;


            // shift all the gears to one side, which is the Y postive side
            double maxshift = (totalThickness - 2 * gap) / 2 - currNum * (thickness + gap); 
            double yOriginal = 0;
            if(maxshift > 0)
            {
                yOriginal -= maxshift;
            }

            List<Point3d> gearCenters = new List<Point3d>();
            double xPos = xEnd;
            double yPos = yOriginal;
            for(int i = 0; i < currNum; i++)
            {
                gearCenters.Add(new Point3d(xPos, yPos, 0));
                xPos -= gearDistance;
                yPos += gap;
                gearCenters.Add(new Point3d(xPos, yPos, 0));
                yPos += thickness;
            }
            Vector3d gearDir = new Vector3d();
            List<Gear> gearEntities = new List<Gear>();
            List<Shape> shaftEntities = new List<Shape>();
            List<Brep> gears = new List<Brep>();
            List<Brep> shafts = new List<Brep>();
            List<Brep> partsForShaftDifference = new List<Brep>();

            foreach(Point3d pt in gearCenters)
            {
                if(gearCenters.IndexOf(pt) %2 == 0)
                {
                    // create the pinion from the output gear end
                    gearDir = gearCenters.ElementAt(gearCenters.IndexOf(pt) + 1) - pt;
                    Gear PinionGear = new Gear(pt, gearDir, teethEqualInt, currModule, pressureAngle,thickness+gap);
                    Brep pinion = PinionGear.Model;

                    gearEntities.Add(PinionGear);

                    #region test by LH
                    //MyDoc.Objects.AddBrep(pinion);
                    //MyDoc.Views.Redraw();
                    #endregion

                    gears.Add(pinion);

                    // create the shaft for the coaxial pinion and big gear

                    // first, get the projected points of the center point on the innershell
                    List<Brep> bs = new List<Brep>();
                    bs.Add(innerShell);
                    List<Point3d> projPts = new List<Point3d>();
                    projPts.Add(pt);
                    Vector3d projDir = new Vector3d(0, 1, 0);
                    Point3d[] projBrepPts;
                    projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

                    if(projBrepPts.Count() == 2)
                    {
                        Point3d pt1 = new Point3d();    // the point on the postive side
                        Point3d pt2 = new Point3d();    // the point on the negative side

                        if(projBrepPts[0].Y > projBrepPts[1].Y)
                        {
                            pt1 = projBrepPts[0] + projDir * 0.2;
                            pt2 = projBrepPts[1] - projDir * 0.2;
                        }
                        else
                        {
                            pt1 = projBrepPts[1] + projDir * 0.2;
                            pt2 = projBrepPts[0] - projDir * 0.2;
                        }
                        Line shaftRailLine = new Line(pt2, pt1);
                        Curve shaftRail = shaftRailLine.ToNurbsCurve();
                        Brep[] shaftRods = Brep.CreatePipe(shaftRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                        Brep shaftRod = shaftRods[0];

                        if (gearCenters.IndexOf(pt) != 0)
                        {
                            // create a brep to different the gears
                            Brep bDiffPart = new Brep();

                            Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            bDiffPart = bDiffParts[0];

                            partsForShaftDifference.Add(bDiffPart);

                            // add two discs around the gear
                            Point3d disc_first_pt1 = pt - projDir * (thickness + tolerance + discThickness);
                            Point3d disc_first_pt2 = pt - projDir * (thickness + tolerance);
                            Point3d disc_second_pt1 = pt + projDir * (thickness + gap + tolerance);
                            Point3d disc_second_pt2 = pt + projDir * (thickness + gap + tolerance + discThickness);

                            Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
                            Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
                            Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
                            Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
                            Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            Brep firstDisc = firstDiscs[0];
                            Brep secondDisc = secondDiscs[0];

                            var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
                            if (shaftAllParts != null)
                            {
                                shafts.Add(shaftAllParts[0]);
                                Shape shaftShape = new Shape(shaftAllParts[0],false,"shaft");
                                shaftEntities.Add(shaftShape);
                            }

                        }
                        else
                        {
                            // only for the output gear

                            if (outputAxle == 1)
                            {
                                // the output gear shaft stay in the object
                                //Point3d closePt = pt.DistanceTo(projBrepPts[0]) > pt.DistanceTo(projBrepPts[1]) ? projBrepPts[1] : projBrepPts[0];
                                //Point3d shSt = pt.Y > closePt.Y ? closePt : pt;
                                //Point3d shEnd = pt.Y > closePt.Y ? (pt + projDir * 4) : (closePt + projDir * 4);

                                //Line sfRailLine = new Line(shSt, shEnd);
                                //Curve sfRail= sfRailLine.ToNurbsCurve();
                                //Brep[] shRods = Brep.CreatePipe(sfRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                //Brep shRod = shRods[0];

                                //shafts.Add(shRod);

                                // create a brep to different the gears
                                Brep bDiffPart = new Brep();

                                Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                bDiffPart = bDiffParts[0];

                                partsForShaftDifference.Add(bDiffPart);

                                // add two discs around the gear
                                Point3d disc_first_pt1 = pt - projDir * (tolerance + discThickness);
                                Point3d disc_first_pt2 = pt - projDir * (tolerance);
                                Point3d disc_second_pt1 = pt + projDir * (thickness + gap + tolerance);
                                Point3d disc_second_pt2 = pt + projDir * (thickness + gap + tolerance + discThickness);

                                Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
                                Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
                                Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
                                Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
                                Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                Brep firstDisc = firstDiscs[0];
                                Brep secondDisc = secondDiscs[0];

                                var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
                                if (shaftAllParts != null)
                                {
                                    shafts.Add(shaftAllParts[0]);
                                    Shape shaftShape = new Shape(shaftAllParts[0], false, "shaft");
                                    shaftEntities.Add(shaftShape);
                                }

                            }
                            else
                            {
                                // the output gear shaft extend out of the object

                                Point3d newPt2 = pt2 - projDir * 4;
                                Point3d newPt1 = pt1 + projDir * 4;

                                Line shRailLine = new Line(newPt2, newPt1);
                                Curve shRail = shRailLine.ToNurbsCurve();
                                Brep[] shRods = Brep.CreatePipe(shRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                Brep shRod = shRods[0];

                                shafts.Add(shRod);
                                Shape shaftShape = new Shape(shRod, false, "shaft");
                                shaftEntities.Add(shaftShape);

                                // difference with the model brep
                                Brep[] shDiffRods = Brep.CreatePipe(shRail, shaftRadius + 0.8, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                                Brep shDiffRod = shDiffRods[0];

                                var outputAxleModelBreps = Brep.CreateBooleanDifference(Model, shDiffRod, MyDoc.ModelAbsoluteTolerance);
                                if (outputAxleModelBreps == null)
                                {
                                    shDiffRod.Flip();
                                    outputAxleModelBreps = Brep.CreateBooleanDifference(Model, shDiffRod, MyDoc.ModelAbsoluteTolerance);
                                }
                                Brep outputAxleModelBrep = outputAxleModelBreps[0];
                                Model = outputAxleModelBrep.DuplicateBrep();
                            }
                            
                        }
                    }

                }
                else
                {
                    // create the big gear 
                    gearDir = pt - gearCenters.ElementAt(gearCenters.IndexOf(pt) - 1);
                    Gear BigGear = new Gear(pt, gearDir, gearTeethNum, currModule, pressureAngle, thickness);
                    Brep bigGear = BigGear.Model;

                    

                    #region test by LH
                    //MyDoc.Objects.AddBrep(bigGear);
                    //MyDoc.Views.Redraw();
                    #endregion

                    //Vector3d planeVec = new Vector3d(1, 0, 0);
                    //Transform directionRotate = Transform.Rotation(new Vector3d(0, 1, 0), gearDir, new Point3d(0, 0, 0));
                    //planeVec.Transform(directionRotate);
                    //Vector3d normalVec = new Vector3d(planeVec.Y * gearDir.Z - gearDir.Y * planeVec.Z,
                    //    planeVec.Z * gearDir.X - gearDir.Z * planeVec.X,
                    //    planeVec.X * gearDir.Y - gearDir.X * planeVec.Y);
                    double meshAngle = Math.PI / (teethEqualInt * grNew);
                    Transform rotat = Transform.Rotation(meshAngle, new Vector3d(0,-1,0), pt);
                    //if (teethEqualInt % 2 == 0)
                    //{
                        bigGear.Transform(rotat);
                    //}
                    BigGear.SetModel(bigGear);
                    gearEntities.Add(BigGear);
                    gears.Add(bigGear);

                    if(gearCenters.IndexOf(pt) == gearCenters.Count() - 1)
                    {
                        // create the last shaft
                        List<Brep> bs = new List<Brep>();
                        lastGearPos = pt;
                        bs.Add(innerShell);
                        List<Point3d> projPts = new List<Point3d>();
                        projPts.Add(pt);
                        Vector3d projDir = new Vector3d(0, 1, 0);
                        Point3d[] projBrepPts;
                        projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

                        if (projBrepPts.Count() == 2)
                        {
                            Point3d pt1 = new Point3d();    // the point on the postive side
                            Point3d pt2 = new Point3d();    // the point on the negative side

                            if (projBrepPts[0].Y > projBrepPts[1].Y)
                            {
                                pt1 = projBrepPts[0] + projDir * 0.2;
                                pt2 = projBrepPts[1] - projDir * 0.2;
                            }
                            else
                            {
                                pt1 = projBrepPts[1] + projDir * 0.2;
                                pt2 = projBrepPts[0] - projDir * 0.2;
                            }
                            Line shaftRailLine = new Line(pt2, pt1);
                            Curve shaftRail = shaftRailLine.ToNurbsCurve();
                            Brep[] shaftRods = Brep.CreatePipe(shaftRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            Brep shaftRod = shaftRods[0];

                           
                            // create a brep to different the gears
                            Brep bDiffPart = new Brep();

                            Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            bDiffPart = bDiffParts[0];

                            partsForShaftDifference.Add(bDiffPart);

                            // add two discs around the gear
                            Point3d disc_first_pt1 = pt - projDir * (tolerance + discThickness);
                            Point3d disc_first_pt2 = pt - projDir * (tolerance);
                            Point3d disc_second_pt1 = pt + projDir * (thickness + tolerance);
                            Point3d disc_second_pt2 = pt + projDir * (thickness + tolerance + discThickness);

                            Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
                            Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
                            Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
                            Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
                            Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
                            Brep firstDisc = firstDiscs[0];
                            Brep secondDisc = secondDiscs[0];

                            var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
                            if (shaftAllParts != null)
                            {
                                shafts.Add(shaftAllParts[0]);
                                Shape shaftshape = new Shape(shaftAllParts[0], false, "shaft");
                                shaftEntities.Add(shaftshape);
                            }
                        }

                    }
                }
            }

            // Boolean different the gears from partsForShaftDifference to drill the gears except for the output gear

            var finalGears = Brep.CreateBooleanDifference(gears, partsForShaftDifference, MyDoc.ModelAbsoluteTolerance);
            if (finalGears == null)
            {
                foreach(Brep b in partsForShaftDifference)
                {
                    b.Flip();
                }
                finalGears = Brep.CreateBooleanDifference(gears, partsForShaftDifference, MyDoc.ModelAbsoluteTolerance);
            }


            // so far, finalGears has all the gears, shafts has all the gear shafts (w/ or w/out discs)

            foreach(Brep s in shafts)
            {
                //MyDoc.Objects.AddBrep(s);
                //MyDoc.Views.Redraw();

            }

            int idx = 0;
            foreach(Brep g in finalGears)
            {
                //MyDoc.Objects.AddBrep(g);
                //MyDoc.Views.Redraw();
                //result.Add(g);
                gearEntities.ElementAt(idx).SetModel(g);
                idx++;
            }

            foreach(Shape sh in shaftEntities)
            {
                Brep b_origi = sh.GetModelinWorldCoordinate();
                b_origi.Transform(translateBack);
                b_origi.Transform(rotationBack);
                b_origi.Transform(postRotationBack);
                sh.SetModel(b_origi);

                EntityList.Add(sh);
            }
            foreach(Gear g in gearEntities)
            {
                Brep b_origi = g.GetModelinWorldCoordinate();
                b_origi.Transform(translateBack);
                b_origi.Transform(rotationBack);
                b_origi.Transform(postRotationBack);
                g.SetModel(b_origi);

                EntityList.Add(g);
            }

            #endregion

            #endregion

            #region Step 2: construct the spring

            if (this.InputType == 1)
            {
                #region construct a helical spring 
                Vector3d backwardDir = new Vector3d(-1, 0, 0);
                Point3d rackTip = lastGearPos + backwardDir * (teethEqualInt * grNew * currModule / 2 + backlash * backlashGap);

                // get the spring skeleton
                Point3d springPos = rackTip + backwardDir * (2 * currModule + 3 + 1 + springWidthThreadshold/2);     // 3 is the rack thickness, 1 is the gap
                List<Brep> bs = new List<Brep>();
                bs.Add(innerShell);
                List<Point3d> projPts = new List<Point3d>();
                projPts.Add(springPos);
                Vector3d projDir = new Vector3d(0, 0, 1);
                Point3d[] projBrepPts;
                projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

                double totalLen = 0;
                Point3d SpringSt, SpringEnd;

                SpringSt = springPos - projDir * outDiameter / 2.5;
                SpringEnd = springPos + projDir * outDiameter / 2;
                totalLen = SpringSt.DistanceTo(SpringEnd);
                
                Line skeletonLine = new Line(SpringSt, SpringEnd);
                Curve skeletonH = skeletonLine.ToNurbsCurve();

                double s_len = SpringSt.DistanceTo(SpringEnd);
                double min_wire_diamter = 2.8;
                double min_coil_num = 3;
                double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                double displacement = 0.9 * maxDisp/s_len;

                springH = new Helix(SpringSt, SpringEnd, springWidthThreadshold, 2.8, 0, displacement, Energy);

                Brep brep_s_origi = springH.GetModelinWorldCoordinate();
                brep_s_origi.Transform(translateBack);
                brep_s_origi.Transform(rotationBack);
                brep_s_origi.Transform(postRotationBack);
                springH.SetModel(brep_s_origi);

                EntityList.Add(springH);
                #endregion

                #region test by LH
                //MyDoc.Objects.AddBrep(springH.GetModelinWorldCoordinate());
                //MyDoc.Views.Redraw();
                #endregion

                result.Add(springH.GetModelinWorldCoordinate());

                #region construct the rack

                Point3d rackStPt = SpringEnd + projDir * 1 + (-1) * backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1);
                Point3d rackEndPt = SpringSt + projDir * displacement * s_len + (-1) * backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1);
                Rack rack = new Rack(rackStPt, rackEndPt, new Point3d(1, 0, 0), currModule);

                Brep brep_r_origi = rack.GetModelinWorldCoordinate();
                brep_r_origi.Transform(translateBack);
                brep_r_origi.Transform(rotationBack);
                brep_r_origi.Transform(postRotationBack);
                rack.SetModel(brep_r_origi);

                EntityList.Add(rack);

                #endregion

                #region test by LH
                //MyDoc.Objects.AddBrep(rack.GetModelinWorldCoordinate());
                //MyDoc.Views.Redraw();
                #endregion


                // create sweep function
                var sweep = new Rhino.Geometry.SweepOneRail();
                sweep.AngleToleranceRadians = MyDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = MyDoc.ModelAbsoluteTolerance;


                #region construct connectors and control button

                #region construct the upper plate

                Point3d upperPlateSt = SpringEnd;
                Point3d upperPlateEnd = SpringEnd + projDir * 2;
                Plane upperPlatePln = new Plane(upperPlateSt, projDir);

                Vector3d up_xp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * upperPlatePln.XAxis;
                Vector3d up_xn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * upperPlatePln.XAxis;
                Vector3d up_yp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * upperPlatePln.YAxis;
                Vector3d up_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * upperPlatePln.YAxis;

                Point3d[] upperPlatePts = new Point3d[5];
                upperPlatePts[0] = upperPlateSt + up_xp + up_yp;
                upperPlatePts[1] = upperPlateSt + up_xn + up_yp;
                upperPlatePts[2] = upperPlateSt + up_xn + up_yn;
                upperPlatePts[3] = upperPlateSt + up_xp + up_yn;
                upperPlatePts[4] = upperPlateSt + up_xp + up_yp;
                Curve upperPlateRect = new Polyline(upperPlatePts).ToNurbsCurve();

                
                Point3d[] upperRailPts = new Point3d[2];
                upperRailPts[0] = upperPlateSt;
                upperRailPts[1] = upperPlateEnd;
                Curve rail1 = new Polyline(upperRailPts).ToNurbsCurve();

                Brep upperPlateBrep = new Brep();

                upperPlateBrep = sweep.PerformSweep(rail1, upperPlateRect)[0];
                upperPlateBrep = upperPlateBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

                #endregion

                #region constrcut the base plate

                Point3d bottomPlateSt = projBrepPts[0].Z > projBrepPts[1].Z?projBrepPts[1]:projBrepPts[0];
                Point3d bottomPlateEnd = SpringSt;
                Plane bottomPlatePln = new Plane(bottomPlateSt, projDir);

                Vector3d b_xp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * bottomPlatePln.XAxis;
                Vector3d b_xn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * bottomPlatePln.XAxis;
                Vector3d b_yp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * bottomPlatePln.YAxis;
                Vector3d b_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * bottomPlatePln.YAxis;

                Point3d[] bottomPlatePts = new Point3d[5];
                bottomPlatePts[0] = bottomPlateSt + b_xp + b_yp;
                bottomPlatePts[1] = bottomPlateSt + b_xn + b_yp;
                bottomPlatePts[2] = bottomPlateSt + b_xn + b_yn;
                bottomPlatePts[3] = bottomPlateSt + b_xp + b_yn;
                bottomPlatePts[4] = bottomPlateSt + b_xp + b_yp;
                Curve bottomPlateRect = new Polyline(bottomPlatePts).ToNurbsCurve();


                Point3d[] bottomRailPts = new Point3d[2];
                bottomRailPts[0] = bottomPlateSt;
                bottomRailPts[1] = bottomPlateEnd;
                Curve rail2 = new Polyline(bottomRailPts).ToNurbsCurve();

                Brep bottomPlateBrep = new Brep();

                bottomPlateBrep = sweep.PerformSweep(rail2, bottomPlateRect)[0];
                bottomPlateBrep = bottomPlateBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

                #endregion

                #region constrcut the button

                Point3d btnSt = upperPlateEnd;
                Point3d btnEnd = projBrepPts[0].Z > projBrepPts[1].Z ? projBrepPts[0] + (1.6 + displacement * s_len) * projDir : projBrepPts[1] + (1.6 + displacement * s_len) * projDir;

                Point3d[] btnRailPts = new Point3d[2];
                btnRailPts[0] = btnSt;
                btnRailPts[1] = btnEnd;
                Curve rail3 = new Polyline(btnRailPts).ToNurbsCurve();

                Brep btnHandlerBrep = new Brep();
                Brep btnHandlerDiffBrep = new Brep();
                Brep btnTipBrep = new Brep();

                btnHandlerBrep = Brep.CreatePipe(rail3, 2.5, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
                btnHandlerDiffBrep = Brep.CreatePipe(rail3, 3.5, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];


                Point3d[] btnTipRailPts = new Point3d[2];
                btnTipRailPts[0] = btnEnd;
                btnTipRailPts[1] = btnEnd + projDir * 4;
                Curve rail4 = new Polyline(btnTipRailPts).ToNurbsCurve();
                btnTipBrep = Brep.CreatePipe(rail4, 10, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

                var finalModelBreps = Brep.CreateBooleanDifference(Model, btnHandlerDiffBrep, MyDoc.ModelAbsoluteTolerance);
                if (finalModelBreps == null)
                {
                    btnHandlerDiffBrep.Flip();
                    finalModelBreps = Brep.CreateBooleanDifference(Model, btnHandlerDiffBrep, MyDoc.ModelAbsoluteTolerance);
                }
                Brep finalModelBrep = finalModelBreps[0];
                Model = finalModelBrep.DuplicateBrep();

                #endregion

                #region test by LH

                //MyDoc.Objects.AddBrep(upperPlateBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(bottomPlateBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(btnHandlerBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(btnTipBrep);
                //MyDoc.Views.Redraw();

                #endregion

                upperPlateBrep.Transform(translateBack);
                upperPlateBrep.Transform(rotationBack);
                upperPlateBrep.Transform(postRotationBack);
                Shape upperPlateShape = new Shape(upperPlateBrep, false, "connector");

                bottomPlateBrep.Transform(translateBack);
                bottomPlateBrep.Transform(rotationBack);
                bottomPlateBrep.Transform(postRotationBack);
                Shape bottomPlateShape = new Shape(bottomPlateBrep, false, "connector");

                btnHandlerBrep.Transform(translateBack);
                btnHandlerBrep.Transform(rotationBack);
                btnHandlerBrep.Transform(postRotationBack);
                Shape btnHandlerShape = new Shape(btnHandlerBrep, false, "connector");

                btnTipBrep.Transform(translateBack);
                btnTipBrep.Transform(rotationBack);
                btnTipBrep.Transform(postRotationBack);
                Shape btnTipShape = new Shape(btnTipBrep, false, "connector");

                EntityList.Add(upperPlateShape);
                EntityList.Add(bottomPlateShape);
                EntityList.Add(btnHandlerShape);
                EntityList.Add(btnTipShape);

                #endregion


                #region construct the guide for the spring

                #region constrcut the hookbar and the hookguide
                Point3d barEnd = upperPlateSt + backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.5) + projDir * 2;
                Point3d barSt = barEnd - projDir * (totalLen - displacement * s_len - 4);
                Point3d guideSt = barSt + projDir * (4 + 0.2);        // the hook height is 6mm
                Point3d guideEnd = guideSt + backwardDir * (4 + 1.4);

                Plane hookBarPln = new Plane(barSt, projDir);

                Vector3d hb_xp = 0.6 * hookBarPln.XAxis;
                Vector3d hb_xn = -0.6* hookBarPln.XAxis;
                Vector3d hb_yp = 1.2 * hookBarPln.YAxis;
                Vector3d hb_yn = -1.2 * hookBarPln.YAxis;

                Point3d[] hookBarPts = new Point3d[5];
                hookBarPts[0] = barSt + hb_xp + hb_yp;
                hookBarPts[1] = barSt + hb_xn + hb_yp;
                hookBarPts[2] = barSt + hb_xn + hb_yn;
                hookBarPts[3] = barSt + hb_xp + hb_yn;
                hookBarPts[4] = barSt + hb_xp + hb_yp;
                Curve hookBarRect = new Polyline(hookBarPts).ToNurbsCurve();


                Point3d[] hookBarRailPts = new Point3d[2];
                hookBarRailPts[0] = barSt;
                hookBarRailPts[1] = barEnd;
                Curve rail5 = new Polyline(hookBarRailPts).ToNurbsCurve();

                Brep hookbarBrep = new Brep();

                hookbarBrep = sweep.PerformSweep(rail5, hookBarRect)[0];
                hookbarBrep = hookbarBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);


                Point3d[] hookGuideRailPts = new Point3d[2];
                hookGuideRailPts[0] = guideSt;
                hookGuideRailPts[1] = guideEnd;
                Curve rail6 = new Polyline(hookGuideRailPts).ToNurbsCurve();

                Brep hookGuideBrep = new Brep();

                hookGuideBrep = Brep.CreatePipe(rail6, 0.8, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

                #endregion

                #region construct the guide
                Point3d guideWallPos = SpringSt + backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.5 + 4);

                List<Brep> bs_wall = new List<Brep>();
                bs_wall.Add(innerShell);
                List<Point3d> projWallPts = new List<Point3d>();
                projWallPts.Add(guideWallPos);
                Vector3d projWallDir = new Vector3d(0, 0, 1);
                Point3d[] projWallBrepPts;
                projWallBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs_wall, projWallPts, projWallDir, MyDoc.ModelAbsoluteTolerance);

                Point3d guideWallSt, guideWallEnd;

                if(projWallBrepPts[0].Z > projWallBrepPts[1].Z)
                {
                    guideWallEnd = projWallBrepPts[0];
                    guideWallSt = projWallBrepPts[1];
                }
                else
                {
                    guideWallEnd = projWallBrepPts[1];
                    guideWallSt = projWallBrepPts[0];
                }

                Plane guideWallPln = new Plane(guideWallSt, projWallDir);

                Vector3d gw_xp = 0.8 * guideWallPln.XAxis;
                Vector3d gw_xn = -0.8 * guideWallPln.XAxis;
                Vector3d gw_yp = ((springWidthThreadshold + springH.WireRadius) / 2) * guideWallPln.YAxis;
                Vector3d gw_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2) * guideWallPln.YAxis;

                Point3d[] guideWallPts = new Point3d[5];
                guideWallPts[0] = guideWallSt + gw_xp + gw_yp;
                guideWallPts[1] = guideWallSt + gw_xn + gw_yp;
                guideWallPts[2] = guideWallSt + gw_xn + gw_yn;
                guideWallPts[3] = guideWallSt + gw_xp + gw_yn;
                guideWallPts[4] = guideWallSt + gw_xp + gw_yp;
                Curve guideWallRect = new Polyline(guideWallPts).ToNurbsCurve();


                Point3d[] guideWallRailPts = new Point3d[2];
                guideWallRailPts[0] = guideWallSt;
                guideWallRailPts[1] = guideWallEnd;
                Curve rail7 = new Polyline(guideWallRailPts).ToNurbsCurve();

                Brep guideWallBrep = new Brep();

                guideWallBrep = sweep.PerformSweep(rail7, guideWallRect)[0];
                guideWallBrep = guideWallBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

                // Open the linear slot 

                Point3d slotSt = guideWallPos;
                Point3d slotEnd = guideWallPos + projDir * (totalLen + 2);

                Plane wallSlotPln = new Plane(slotSt, projDir);

                Vector3d ws_xp = 2 * wallSlotPln.XAxis;
                Vector3d ws_xn = -2 * wallSlotPln.XAxis;
                Vector3d ws_yp = 1.4 * wallSlotPln.YAxis;
                Vector3d ws_yn = -1.4 * wallSlotPln.YAxis;

                Point3d[] wallSlotPts = new Point3d[5];
                wallSlotPts[0] = slotSt + ws_xp + ws_yp;
                wallSlotPts[1] = slotSt + ws_xn + ws_yp;
                wallSlotPts[2] = slotSt + ws_xn + ws_yn;
                wallSlotPts[3] = slotSt + ws_xp + ws_yn;
                wallSlotPts[4] = slotSt + ws_xp + ws_yp;
                Curve wallSlotRect = new Polyline(wallSlotPts).ToNurbsCurve();


                Point3d[] wallSlotRailPts = new Point3d[2];
                wallSlotRailPts[0] = slotSt;
                wallSlotRailPts[1] = slotEnd;
                Curve rail8 = new Polyline(wallSlotRailPts).ToNurbsCurve();

                Brep wallSlotBrep = new Brep();

                wallSlotBrep = sweep.PerformSweep(rail8, wallSlotRect)[0];
                wallSlotBrep = wallSlotBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

                var wallGuideFinalBreps = Brep.CreateBooleanDifference(guideWallBrep, wallSlotBrep, MyDoc.ModelAbsoluteTolerance);
                if (wallGuideFinalBreps == null)
                {
                    wallSlotBrep.Flip();
                    wallGuideFinalBreps = Brep.CreateBooleanDifference(guideWallBrep, wallSlotBrep, MyDoc.ModelAbsoluteTolerance);
                }
                Brep wallGuideFinalBrep = wallGuideFinalBreps[0];

                #endregion

                #region test by LH

                //MyDoc.Objects.AddBrep(hookbarBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(hookGuideBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(wallGuideFinalBrep);
                //MyDoc.Views.Redraw();
                //MyDoc.Objects.AddBrep(Model);
                //MyDoc.Views.Redraw();

                #endregion

                hookbarBrep.Transform(translateBack);
                hookbarBrep.Transform(rotationBack);
                hookbarBrep.Transform(postRotationBack);
                Shape hookbarShape = new Shape(hookbarBrep, false, "connector");

                hookGuideBrep.Transform(translateBack);
                hookGuideBrep.Transform(rotationBack);
                hookGuideBrep.Transform(postRotationBack);
                Shape hookGuideShape = new Shape(hookGuideBrep, false, "connector");

                wallGuideFinalBrep.Transform(translateBack);
                wallGuideFinalBrep.Transform(rotationBack);
                wallGuideFinalBrep.Transform(postRotationBack);
                Shape wallGuideFinalShape = new Shape(wallGuideFinalBrep, false, "connector");

                Shape ModelShape = new Shape(Model, false, "connector");

                EntityList.Add(hookbarShape);
                EntityList.Add(hookGuideShape);
                EntityList.Add(wallGuideFinalShape);
                EntityList.Add(ModelShape);

                #endregion

            }
            else
            {
                // construct a spiral spring 

            }
            #endregion


        }


        #region Member viarables encapsulations
        public List<Lock> Locks { get => locks; set => locks = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }
        public Curve Skeleton { get => skeleton; set => skeleton = value; }
        public Brep Model { get => _model; set => _model = value; }
        public double Energy { get => _energy; set => _energy = value; }
        public double Speed { get => _speed; set => _speed = value; }
        public Vector3d Direction { get => _direction; set => _direction = value; }
        public bool AddLock { get => _addLock; set => _addLock = value; }
        public RhinoDoc MyDoc { get => _myDoc; set => _myDoc = value; }
        public int InputType { get => _inputType; set => _inputType = value; }
        public Brep InnerCavity { get => _innerCavity; set => _innerCavity = value; }
        public Helix SpringH { get => springH; set => springH = value; }
        public Spiral SpringS { get => springS; set => springS = value; }
        #endregion
    }
}
