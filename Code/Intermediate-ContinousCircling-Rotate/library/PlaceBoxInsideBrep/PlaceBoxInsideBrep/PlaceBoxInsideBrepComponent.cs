using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace PlaceBoxInsideBrep
{
    public class PlaceBoxInsideBrepComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public PlaceBoxInsideBrepComponent()
          : base("PlaceBoxInsideBrep", "PlaceBoxInsideBrep",
              "Place a Box Inside a Brep.",
              "Brep", "Kinetic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "bp", "Brep To Place Box", GH_ParamAccess.item);
            pManager.AddBrepParameter("BrepList", "Me", "Mechanism Box", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("Int32", "int","Index of Gear In the box", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector3d", "vec", "Direction of Movement", GH_ParamAccess.item);
            pManager.AddCurveParameter("Curve", "curve", "SpiralLine", GH_ParamAccess.item);
            pManager.AddCurveParameter("Curve", "arrow", "arrow to show in rotation", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "axis","objects on axis which will rotate with the main gear", GH_ParamAccess.list);
            pManager.AddBrepParameter("Brep", "bottom", "bottom support which will move in simulation", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "gear", "main gear", GH_ParamAccess.item);
            pManager.AddCurveParameter("Curve", "spiral", "curve of the spiral", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector3d", "GearNormal", "normal of the gear", GH_ParamAccess.item);
        }

        private BoundingBox FindBoundaryBox(List<Brep> breps)
        {
            BoundingBox b = breps[0].GetBoundingBox(false);
            double xMin = b.Min.X;
            double yMin = b.Min.Y;
            double zMin = b.Min.Z;
            double xMax = b.Max.X;
            double yMax = b.Max.Y;
            double zMax = b.Max.Z;
            int i = 1;
            while (i < breps.Count)
            {
                b = breps[i].GetBoundingBox(false);
                if (b.Min.X < xMin)
                    xMin = b.Min.X;
                if (b.Min.Y < yMin)
                    yMin = b.Min.Y;
                if (b.Min.Z < zMin)
                    zMin = b.Min.Z;
                if (xMax < b.Max.X)
                    xMax = b.Max.X;
                if (yMax < b.Max.Y)
                    yMax = b.Max.Y;
                if (zMax < b.Max.Z)
                    zMax = b.Max.Z;
                i++;
            }
            return new BoundingBox(xMin,yMin,zMin,xMax,yMax,zMax);
        }

        /*
        private Brep AddKnob(BoundingBox box)
        {
            //Add an knob shape to the box
            Curve c = new Circle(box.Center, 1).ToNurbsCurve();
            Brep brep = Brep.CreatePlanarBreps(c, Rhino.RhinoMath.SqrtEpsilon)[0];
            BrepFace surface = brep.Faces[0];
            double length = 20;
            Curve GearThickness = new Line(c.PointAtStart, length * Vector3d.ZAxis).ToNurbsCurve();
            Brep Cylinder = surface.CreateExtrusion(GearThickness, true);
            return Cylinder;
        }
        */

        private bool BoxInBrep(double x, double y, double z, double xb, double yb, double zb, Brep bp,double length)
        {
            BoundingBox bbox = new BoundingBox(x - xb / 2, y - yb / 2, z - zb / 2, x + xb / 2, y + yb / 2, z + zb / 2);
            Brep box = new Box(bbox).ToBrep();
            Curve[] curves;
            Point3d[] points;
            Rhino.Geometry.Intersect.Intersection.BrepBrep(box, bp, RhinoMath.SqrtEpsilon, out curves, out points);
            if (curves.Length>0)
                return false;
            else
            {
                Curve l = new Line(bbox.Center, Vector3d.XAxis, length * 2).ToNurbsCurve();
                l.Transform(Transform.Translation(-Vector3d.XAxis * length));
                double[] t;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(l, bp, RhinoMath.SqrtEpsilon, RhinoMath.DefaultAngleTolerance, out t);
                if (t.Length==0)
                    return false;
                else
                {
                    int sideLeft = 0;
                    int sideRight = 0;
                    foreach (double tp in t)
                    {
                        if (l.PointAt(tp).X - x < 0)
                            sideLeft += 1;
                        else
                            sideRight += 1;
                    }
                    if (sideLeft == 0 || sideRight == 0)
                        return false;
                }
            }
            return true;                     
        }

        private List<Point3d> AvailableCentroid(BoundingBox b, double xb, double yb, double zb, Brep bp,double length)
        {
            //Find initial points
            double xp = b.Min.X + xb / 2 + 1;
            double yp = b.Min.Y + yb / 2 + 1;
            double zp = b.Min.Z + zb / 2 + 1;
            double gridSize = 5.0;
            //Output Params
            List<Point3d> pl = new List<Point3d>();
            //Point3d pd = b.Min;
            //double dist = brepCentre.DistanceTo(pd);
            //Transform during iteration
            for (double x = xp; x < b.Max.X - xb / 2 - 1; x += gridSize)
            {
                for (double y = yp; y < b.Max.Y - yb / 2 - 1; y += gridSize)
                {
                    for (double z = zp; z < b.Max.Z - zb / 2 - 1; z += gridSize)
                    {
                        if (BoxInBrep(x, y, z, xb, yb, zb, bp,length))
                        {
                            Point3d p = new Point3d(x, y, z);
                            pl.Add(p);
                            /*
                            double distance = brepCentre.DistanceTo(p);
                            if (distance < dist)
                            {
                                dist = distance;
                                pd = p;
                            }
                            */

                        }

                    }
                }
            }
            return pl;
        }

        private List<Point3d> FindRotation(Brep bp, BoundingBox box, Vector3d KnobDirection,double length)
        {
            List<Point3d> Rotation = new List<Point3d>();
            Point3d center = box.Center;
            double x = center.X;
            double y = center.Y;
            double z = center.Z;
            double xb = box.Max.X - box.Min.X;
            double yb = box.Max.Y - box.Min.Y;
            double zb = box.Max.Z - box.Min.Z;
            double dist = 50.0;
            if (KnobDirection == Vector3d.YAxis)
            {
                Rotation.Add(new Point3d(x, y + dist, z));
                Rotation.Add(new Point3d(x, y - dist, z));
                /*
                if (BoxInBrep(x, y, z, yb, xb, zb, bp))
                {
                    Rotation.Add(new Point3d(x + dist, y, z));
                    Rotation.Add(new Point3d(x - dist, y, z));
                }
                */
                if (BoxInBrep(x, y, z, xb, zb, yb, bp,length))
                {
                    Rotation.Add(new Point3d(x, y, z + dist));
                    Rotation.Add(new Point3d(x, y, z - dist));
                }
            }
            else
            {
                Rotation.Add(new Point3d(x, y, z + dist));
                Rotation.Add(new Point3d(x, y, z - dist));
                /*
                if (BoxInBrep(x, y, z, zb, yb, xb, bp))
                {
                    Rotation.Add(new Point3d(x + dist, y, z));
                    Rotation.Add(new Point3d(x - dist, y, z));
                }
                */
                if (BoxInBrep(x, y, z, xb, zb, yb, bp,length))
                {
                    Rotation.Add(new Point3d(x, y + dist, z));
                    Rotation.Add(new Point3d(x, y - dist, z));
                }
            }
            /*
            else
            {
                Rotation.Add(new Point3d(x + dist, y, z));
                Rotation.Add(new Point3d(x - dist, y, z));
                if (BoxInBrep(x, y, z, zb, yb, xb, bp))
                {
                    Rotation.Add(new Point3d(x, y, z + dist));
                    Rotation.Add(new Point3d(x, y, z - dist));
                }
                if (BoxInBrep(x, y, z, yb, xb, zb, bp))
                {
                    Rotation.Add(new Point3d(x, y + dist, z));
                    Rotation.Add(new Point3d(x, y - dist, z));
                }
            }
            */
            return Rotation;            
        }

        private Brep CreateAxis(Point3d bottomcenter, Plane circlePlane, double length,double radius)
        {
            //Add an knob shape to the box
            Curve c = new Circle(circlePlane, bottomcenter, radius).ToNurbsCurve();
            Brep brep = Brep.CreatePlanarBreps(c, Rhino.RhinoMath.SqrtEpsilon)[0];
            BrepFace surface = brep.Faces[0];
            Curve GearThickness = new Line(c.PointAtStart, length * circlePlane.Normal).ToNurbsCurve();
            Brep Cylinder = surface.CreateExtrusion(GearThickness, true);
            return Cylinder;
        }

        private List<Brep> CreateAxisAndknob(Brep bp,Point3d centroid, Point3d rotation, Brep bottom,double yp,double zp, Point3d spiralEnd)
        {
            List<Brep> breps = new List<Brep>();
            BoundingBox box = bp.GetBoundingBox(false);
            Curve c0;
            Curve c1;
            if (Math.Abs(rotation.Y - centroid.Y) > 1)
            {
                Point3d from = new Point3d(rotation.X, box.Max.Y + 10.0, rotation.Z);
                Point3d to = new Point3d(rotation.X, box.Min.Y - 10.0, rotation.Z);
                c0 = new Line(from, to).ToNurbsCurve();
                c1 = new Line(c0.PointAtStart,c0.PointAtEnd).ToNurbsCurve();
                c1.Transform(Transform.Translation(spiralEnd - centroid));
            }
            else
            {
                Point3d from = new Point3d(rotation.X, rotation.Y, box.Max.Z + 10.0);
                Point3d to = new Point3d(rotation.X, rotation.Y, box.Min.Z - 10.0);
                c0 = new Line(from, to).ToNurbsCurve();
                c1 = new Line(c0.PointAtStart, c0.PointAtEnd).ToNurbsCurve();
                c1.Transform(Transform.Translation(spiralEnd - centroid));
            }
            double[] t0;
            double[] t1;
            Rhino.Geometry.Intersect.Intersection.CurveBrep(c0, bp, RhinoMath.SqrtEpsilon, RhinoMath.SqrtEpsilon, out t0);
            Rhino.Geometry.Intersect.Intersection.CurveBrep(c1, bp, RhinoMath.SqrtEpsilon, RhinoMath.SqrtEpsilon, out t1);
            Point3d p1 = c0.PointAt(t0[0]);
            Point3d p2 = c0.PointAt(t0[1]);
            Point3d p3 = c1.PointAt(t1[0]);
            Point3d p4 = c1.PointAt(t1[1]);
            if(p1.DistanceTo(rotation) >p2.DistanceTo(rotation))
            {
                p1 = c0.PointAt(t0[1]);
                p2 = c0.PointAt(t0[0]);
            }
            if (p3.DistanceTo(rotation) > p4.DistanceTo(rotation))
            {
                p3 = c1.PointAt(t0[1]);
                p4 = c1.PointAt(t0[0]);
            }
            //Dimension of axises and knobs
            double BottomAxisLength = 1;
            double BottomAxisRadius = 5;
            double MidAxisLength = p1.DistanceTo(p2) + 10;
            double MidAxisRadius = 2;
            double KnobAxisLength = 2;
            double KnobAxisRadius = 5;
            double spiralSupportRadius = 1;
            BoundingBox bottomBox = bottom.GetBoundingBox(false);
            if (rotation.Y - centroid.Y > 1)
            {
                var xf = Transform.Translation(Vector3d.YAxis * (p2.Y - bottomBox.Min.Y));
                bottom.Transform(xf);
                //Create Axis
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y + 2, p2.Z), Plane.WorldZX, BottomAxisLength, BottomAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y + 2 + BottomAxisLength, p2.Z), Plane.WorldZX, MidAxisLength, MidAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y + 2 + BottomAxisLength + MidAxisLength, p2.Z), Plane.WorldZX, KnobAxisLength, KnobAxisRadius));
                breps.Add(CreateAxis(p4, Plane.WorldZX, p4.DistanceTo(p3), spiralSupportRadius));
            }
            else if(rotation.Y-centroid.Y<-1)
            {
                var xf = Transform.Translation(Vector3d.YAxis * (p2.Y - bottomBox.Max.Y));
                bottom.Transform(xf);
                //Create Axis
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y - 2, p2.Z),Plane.WorldZX,-BottomAxisLength, BottomAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y - 2 - BottomAxisLength, p2.Z),Plane.WorldZX,-MidAxisLength, MidAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y - 2 - BottomAxisLength - MidAxisLength, p2.Z), Plane.WorldZX,-KnobAxisLength, KnobAxisRadius));
                breps.Add(CreateAxis(p4, Plane.WorldZX, -p4.DistanceTo(p3), spiralSupportRadius));
            }
            else if(rotation.Z - centroid.Z > 1)
            {
                var xf = Transform.Translation(Vector3d.ZAxis * (p2.Z - bottomBox.Min.Z));
                bottom.Transform(xf);
                //Create Axis
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z + 2), Plane.WorldXY, BottomAxisLength, BottomAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z + 2 + BottomAxisLength), Plane.WorldXY, MidAxisLength, MidAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z + 2 + BottomAxisLength + MidAxisLength), Plane.WorldXY, KnobAxisLength, KnobAxisRadius));
                breps.Add(CreateAxis(p4, Plane.WorldXY, p4.DistanceTo(p3), spiralSupportRadius));
            }
            else
            {
                var xf = Transform.Translation(Vector3d.ZAxis * (p2.Z - bottomBox.Max.Z));
                bottom.Transform(xf);
                //Create Axis
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z - 2), Plane.WorldXY, -BottomAxisLength, BottomAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z - 2 - BottomAxisLength), Plane.WorldXY, -MidAxisLength, MidAxisRadius));
                breps.Add(CreateAxis(new Point3d(p2.X, p2.Y, p2.Z - 2 - BottomAxisLength - MidAxisLength), Plane.WorldXY, -KnobAxisLength, KnobAxisRadius));
                breps.Add(CreateAxis(p4, Plane.WorldXY, -p4.DistanceTo(p3), spiralSupportRadius));
            }
            breps.Add(bottom);
            return breps;
        }

        private Curve CreateArrow(Curve arrow,Point3d p, Point3d center,Transform xrotateBack)
        {
            arrow.Transform(Transform.Translation(p.X, p.Y, p.Z));
            arrow.Transform(Transform.Rotation(Vector3d.XAxis, p - center, p));
            //Draw arrow on each point
            arrow.Transform(xrotateBack);
            return arrow;
        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare a variable for the input brep and box;
            Brep bp = null;
            List<Brep> brepList = new List<Brep>();
            //Int32 index = 0;
            Vector3d direct = new Vector3d();
            Curve spiral = null;
            Curve arrow = null;
            //Use the DA object to retrieve the data inside the first input parameter.
            //If the retrieval fails we need to abort.
            if (!DA.GetData(0, ref bp )) { return; }
            if (!DA.GetDataList(1, brepList)) { return; }
            //if(!DA.GetData(2, ref index)) { return; }
            if(!DA.GetData(2,ref direct)) { return; }
            if (!DA.GetData(3, ref spiral)) { return; }
            if(!DA.GetData(4,ref arrow)) { return; }
            //Abort on invalid inputs
            if (!bp.IsValid || !bp.IsSolid) { return; }
            if (brepList==null) { return; }
            //Rotate all the objects to X direction
            var xrotate = Transform.Rotation(direct, Vector3d.XAxis, Point3d.Origin);
            var xrotateBack = Transform.Rotation(Vector3d.XAxis, direct, Point3d.Origin);
            //Transform brep in model to code
            bp.Transform(xrotate);
            //Get The XYZ length of the bounding Box of the brep
            BoundingBox b = bp.GetBoundingBox(true);
            double xp = b.Max.X - b.Min.X;
            double yp = b.Max.Y - b.Min.Y;
            double zp = b.Max.Z - b.Min.Z;
            Point3d brepCentre = b.Center;
            double length = b.Min.DistanceTo(b.Max);
            //Get The XYZ length of the Box
            BoundingBox box = FindBoundaryBox(brepList);
            double xb = box.Max.X - box.Min.X;
            double yb = box.Max.Y - box.Min.Y;
            double zb = box.Max.Z - box.Min.Z;
            //rep knob = AddKnob(box);
            //Output
            List<Brep> Axis =new List<Brep>();
            Brep bottom = null;
            List<Point3d> pl = new List<Point3d>();
            Point3d centroid = Point3d.Unset;
            RhinoList<Guid> IdList = new RhinoList<Guid>();
            Vector3d KnobDirection = Vector3d.YAxis;
            Vector3d gearNormal=Vector3d.Unset;

             //Test whether brep can contain box
            //Place box with normal y
            if (xb < xp && yb < zp && zb < yp)
            {
                pl = AvailableCentroid(b, xb, zb, yb, bp,length);
                if (pl.Count>0)
                {
                    var xr = Transform.Rotation(Math.PI / 2, Vector3d.XAxis, box.Center);
                    box.Transform(xr);
                    spiral.Transform(xr);
                    foreach (Brep brep in brepList)
                    {
                        brep.Transform(xr);
                        //Guid id=myDoc.Objects.AddBrep(brep);
                        //MechanismBox.Add(id);
                    }
                }
            }
            if(pl.Count==0 && xb < xp && yb < yp && zb < zp)
            {
                pl = AvailableCentroid(b, xb, yb, zb, bp,length);
                if(pl.Count>0)
                    KnobDirection = Vector3d.ZAxis;
            }
            /*
            if(pl.Count==0&& zb < xp && yb < yp && xb < zp)
            {            
                pl = AvailableCentroid(b, zb, yb, xb, bp);
                if(pl.Count>0)
                {
                    var xr = Transform.Rotation(Math.PI / 2, Vector3d.YAxis, box.Center);
                    box.Transform(xr);
                    transform *= xr;
                    /*foreach (Brep brep in brepList)
                    {
                        brep.Transform(xr);
                        //Guid id=myDoc.Objects.AddBrep(brep);
                        //MechanismBox.Add(id);
                    }
                    KnobDirection = Vector3d.XAxis;
                }
               
            }
            */
            if(pl.Count==0)
            {
                RhinoApp.WriteLine("The Brep is too small.");
                DA.SetDataList(0, null);
                DA.SetDataList(1, null);
                DA.SetDataList(2, null);
                DA.SetData(3, null);
                return;
            }
            //rotate points back to models
            List<Point3d> plNew = new List<Point3d>();
            ObjectAttributes ob_att = new ObjectAttributes();
            ob_att.ObjectColor = System.Drawing.Color.Red;
            foreach (Point3d p in pl)
            {
                p.Transform(xrotateBack);
                plNew.Add(p);
            }               
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            IdList =myDoc.Objects.AddPoints(plNew,ob_att);
            while (centroid == Point3d.Unset)
            {
                ObjRef obj_ref;
                var rcommand = RhinoGet.GetOneObject("Set the centroid of the box", false, ObjectType.Point, out obj_ref);
                if (rcommand == Rhino.Commands.Result.Success)
                {
                    Guid guid = obj_ref.ObjectId;
                    foreach (Guid g in IdList)
                    {
                        if (g == guid)
                        {
                            centroid = obj_ref.Point().Location;
                            //Rotate point in model to code
                            centroid.Transform(xrotate);
                            break;
                        }
                    }
                    if (centroid == Point3d.Unset)
                        RhinoApp.WriteLine("The point is invalid.");
                }
            }
            var xf = Transform.Translation(centroid - box.Center);
            //box.Transform(xf);
            // RhinoList<Guid> MechanismBox = new RhinoList<Guid>();
            box.Transform(xf);
            spiral.Transform(xf);
            foreach (Brep brep in brepList)
            {
               brep.Transform(xf);
            }
            foreach (Guid g in IdList)
            {
                myDoc.Objects.Delete(g, true);
            }
            //Users are then asked to rotate the box
            Point3d knobPoint = Point3d.Unset;
            List<Point3d> rotate = FindRotation(bp, box, KnobDirection, length);
            //rotate points to model
            RhinoList<Guid> RotationList = new RhinoList<Guid>();

            foreach (Point3d p in rotate)
            {
                RotationList.Add(myDoc.Objects.AddCurve(CreateArrow(arrow,p,centroid,xrotateBack)));
            }           
               
            ObjRef rotation;
            while (knobPoint == Point3d.Unset)
            {
                // Appear Box and Knob when users move the mouse
                var rotatecommand = RhinoGet.GetOneObject("Set the direction of the Knob", false, ObjectType.Curve, out rotation);
                if (rotatecommand == Rhino.Commands.Result.Success)
                {
                    Guid guid_r = rotation.ObjectId;
                    for(int i=0;i<RotationList.Count;i++)
                    {
                        if(guid_r==RotationList[i])
                        {
                            knobPoint = rotate[i];
                            var xr = Transform.Rotation(KnobDirection, knobPoint - box.Center, box.Center);
                            box.Transform(xr);
                            spiral.Transform(xr);
                            foreach (Brep brep in brepList)
                                brep.Transform(xr);
                            //Set connection and axis
                            Axis = CreateAxisAndknob(bp, centroid, knobPoint, brepList[2], yp, zp, spiral.PointAtEnd);
                            bottom = Axis[Axis.Count - 1];
                            Axis.RemoveAt(Axis.Count - 1);
                            if (brepList.Count > 3)
                            {
                                for (int j = 3; j < brepList.Count; j++)
                                    Axis.Add(brepList[j]);
                            }
                            //Rotate output back to model
                            foreach (Brep brep in Axis)
                            {
                                brep.Transform(xrotateBack);
                            }
                            brepList[0].Transform(xrotateBack);
                            bottom.Transform(xrotateBack);

                            //box.Transform(xr);     
                            myDoc.Objects.Delete(RotationList, true);
                            gearNormal = knobPoint - centroid;
                            break;
                        }
                    }
                    if (knobPoint == Point3d.Unset)
                        RhinoApp.WriteLine("The point is invalid.");
                }
            }
            DA.SetDataList(0, Axis);
            DA.SetData(1, bottom);
            DA.SetData(2, brepList[0]);
            DA.SetData(3, spiral);
            DA.SetData(4, gearNormal);
            return;
        }




        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("6fc5cbfc-f424-47af-92aa-80f8137ddacf"); }
        }
    }
}
