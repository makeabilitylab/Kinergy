using System;
using System.Collections.Generic;
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

namespace Kinergy.Utilities
{
    public class UserSelection
    {
        public static int UserSelectPointInRhino(List<Point3d> pts ,RhinoDoc myDoc)
        {
            List<Guid> addedPoints = new List<Guid>();
            foreach(Point3d pt in pts)
            {
                addedPoints.Add(myDoc.Objects.AddPoint(pt));
            }
            ObjRef selectedPoint;
            int selection = -1;
            bool success = false;
            while (success == false)
            {
                var command = RhinoGet.GetOneObject("Please select a Point", true, ObjectType.Point, out selectedPoint);
                Guid guid_r = selectedPoint.ObjectId;

                if (command == Rhino.Commands.Result.Success)
                {
                    for (int i = 0; i < addedPoints.Count(); i++)
                    {
                        if (addedPoints[i] == guid_r)
                        {
                            selection = i;
                            success = true;
                            break;
                        }
                    }
                }
            }
            foreach (Guid g in addedPoints)
            {
                myDoc.Objects.Delete(g, true);
            }
            return selection;
        }
        public static int UserSelectCurveInRhino(List<Curve> crvs ,RhinoDoc myDoc)
        {
            List<Guid> addedCurves = new List<Guid>();
            foreach(Curve c in crvs)
            {
                addedCurves.Add(myDoc.Objects.AddCurve(c));
            }
            ObjRef selectedCurve;
            int selection = -1;
            bool success = false;
            while (success == false)
            {
                var command = RhinoGet.GetOneObject("Please select a curve", true, ObjectType.Curve, out selectedCurve);
                Guid guid_r = selectedCurve.ObjectId;

                if (command == Rhino.Commands.Result.Success)
                {
                    for(int i=0;i<addedCurves.Count();i++)
                    {
                        if(addedCurves[i]==guid_r)
                        {
                            selection=i;
                            success = true;
                            break;
                        }
                    }
                }
            }
            foreach (Guid g in addedCurves)
            {
                myDoc.Objects.Delete(g, true);
            }
            return selection;
        }
    }
}
