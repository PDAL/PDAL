using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using SlimDX;

namespace Flaxen.SlimDXControlLib.MouseExample
{
    public class Trackball
    {
        private float m_windowWidth;
        private float m_windowHeight;
        private Matrix m_previousRotation;

        public Point PreviousPoint;

        public Trackball(float windowWidth, float windowHeight)
        {
            m_windowWidth = windowWidth;
            m_windowHeight = windowHeight;
            PreviousPoint = new Point(0, 0);
            m_previousRotation = Matrix.Identity;
        }

        public Vector3 ProjectTo3D(Point point)
        {
            float mouseX = (float)point.X;
            float mouseY = (float)point.Y;

            // scale bounds to [0,0]..[2,2]
            float x = mouseX / (m_windowWidth / 2.0f);
            float y = mouseY / (m_windowHeight / 2.0f);

            // translate [0,0] to center
            x = x - 1;
            y = y - 1;

            // flip so +Y is up, not down
            y = -y;

            float z2 = 1.0f - x * x - y * y;
            float z = z2 > 0.0f ? (float)Math.Sqrt(z2) : 0.0f;

            return new Vector3(x, y, z);
        }

        public Matrix Update(Point point)
        {
            Vector3 v1 = ProjectTo3D(PreviousPoint);
            v1.Normalize();

            Vector3 v2 = ProjectTo3D(point);
            v2.Normalize();

            Vector3 axis = Vector3.Cross(v1, v2);
            axis.Normalize();

            float theta = AngleBetween(v1, v2);

            Matrix rotation = Matrix.RotationAxis(axis, theta);

            rotation = rotation * m_previousRotation;
            m_previousRotation = rotation;

            PreviousPoint = point;

            return rotation;
        }

        private static float AngleBetween(Vector3 v1, Vector3 v2)
        {
            v1.Normalize();
            v2.Normalize();
            float dot = Vector3.Dot(v1, v2);
            return (float)Math.Acos(dot);
        }

    }
}
