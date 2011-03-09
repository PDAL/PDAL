//-----------------------------------------------------------------------
// <copyright file="Mouse.xaml.cs" company="Flaxen Geo">
//    Copyright (c) 2011, Michael P. Gerlek.  All rights reserved.
// </copyright>
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, 
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of Flaxen Geo nor the names of its contributors may be 
//   used to endorse or promote products derived from this software without 
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
// THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------

namespace Flaxen.SlimDXControlLib.MouseExample
{
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Globalization;
    using System.Windows;
    using System.Windows.Documents;
    using System.Windows.Input;
    using SlimDX;
    using Libpc;
    using System;

    /// <summary>
    /// Implements the MouseExample test.
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private PointCloudRenderEngine m_renderEngine;
        private Point m_downPoint;
        private bool m_isMoving = false;
        private string m_positionString;
        private Trackball m_trackball;

        /// <summary>
        /// Initializes a new instance of the Mouse class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            m_renderEngine = new PointCloudRenderEngine();
            x_contentControl.RegisterRenderer(m_renderEngine);

            this.MouseMove += new MouseEventHandler(Mouse_MouseMove);
            this.MouseDown += new MouseButtonEventHandler(Mouse_MouseDown);
            this.MouseUp += new MouseButtonEventHandler(Mouse_MouseUp);
            this.MouseWheel += new MouseWheelEventHandler(Mouse_MouseWheel);
            
            PositionString = string.Empty;

            this.DataContext = this;

            float minx, miny, minz, maxx, maxy, maxz;

            Vector4[] points = CreateFileData(out minx, out miny, out minz, out maxx, out maxy, out maxz);
            //Vector4[] points = CreateRandomData(out minx, out miny, out minz, out maxx, out maxy, out maxz);

            float rangeX = maxx - minx;
            float rangeY = maxy - miny;
            float rangeZ = maxz - minz;
            float largestRange = Math.Max(rangeX, Math.Max(rangeY, rangeZ));
            m_renderEngine.TranslationVector = new Vector3(-minx, -miny, -minz);
            m_renderEngine.ScaleVector = new Vector3(1 / largestRange, 1 / largestRange, 1 / largestRange);

            m_renderEngine.SetPoints(points, minx, miny, minz, maxx, maxy, maxz);

            m_trackball = null;

            // world coords
            m_renderEngine.CameraPosition = new Vector3(0.5f, 0.5f, -10.0f);
            m_renderEngine.TargetPosition = new Vector3(0.5f, 0.5f, 0.5f);

            return;
        }

        /// <summary>
        /// Gets or sets a string containing the mouse position (in world coordinates).
        /// </summary>
        public string PositionString
        {
            get
            {
                return m_positionString;
            }

            set
            {
                if (value != m_positionString)
                {
                    m_positionString = value;
                    NotifyPropertyChanged("PositionString");
                }
            }
        }

        #region mouse handlers
        private void Mouse_MouseDown(object sender, MouseButtonEventArgs e)
        {
            Point point = e.GetPosition(x_contentControl);

            if (m_trackball == null)
            {
                m_trackball = new Trackball((float)x_contentControl.ActualWidth, (float)x_contentControl.ActualHeight);
            }

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                m_downPoint = point;
                m_trackball.PreviousPoint = m_downPoint;
                m_isMoving = true;
            }
            if (e.RightButton == MouseButtonState.Pressed)
            {
                m_downPoint = point;
                m_isMoving = true;
            }
        }

        private void Mouse_MouseUp(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                m_isMoving = false;
            }
            if (e.RightButton == MouseButtonState.Pressed)
            {
                m_isMoving = false;
            }
        }

        private void Mouse_MouseMove(object sender, MouseEventArgs e)
        {
            Point point = e.GetPosition(x_contentControl);

            {
                Vector3 world = m_renderEngine.ConvertToWorldCoordinates(point, (float)x_contentControl.ActualWidth, (float)x_contentControl.ActualHeight);

                PositionString = string.Format(
                    CultureInfo.InvariantCulture,
                    "[{0},{1}]  X={2}  Y={3}  Z={4}",
                    point.X,
                    point.Y,
                    world.X.ToString("F1", CultureInfo.InvariantCulture),
                    world.Y.ToString("F1", CultureInfo.InvariantCulture),
                    world.Z.ToString("F1", CultureInfo.InvariantCulture));
            }

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                if (m_isMoving)
                {
                    Matrix rotation = m_trackball.Update(point);
                    m_renderEngine.RotationMatrix = rotation;
                }
            }
            if (e.RightButton == MouseButtonState.Pressed)
            {
                if (m_isMoving)
                {
                    float x = (float)(m_downPoint.X - point.X);
                    float y = -(float)(m_downPoint.Y - point.Y);
                    Vector3 vc = m_renderEngine.CameraPosition;
                    Vector3 vt = m_renderEngine.TargetPosition;
                    vc.X += x / 100;
                    vc.Y += y / 100;
                    vt.X += x / 100;
                    vt.Y += y / 100;
                    m_renderEngine.CameraPosition = vc;
                    m_renderEngine.TargetPosition = vt;
                    m_downPoint = point;
                }
            }

            return;
        }

        private void Mouse_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            ////float s = (float)e.Delta / 100;
            ////if (s < 0) s = 1 / -s;

            ////m_renderEngine.Scale *= Matrix.Scaling(s,s,s);
            //m_renderEngine.Scale = m_trackball.Scale(e.Delta);

            Vector3 camera = m_renderEngine.CameraPosition;
            camera.Z += e.Delta / (120.0f * 1);
            if (camera.Z >= m_renderEngine.TargetPosition.Z) return;
            m_renderEngine.CameraPosition = camera;
            return;
        }
        #endregion

        private DecimationFilter m_filter = null; // BUG: swig scoping issue
        private LiblasReader m_reader = null;

        private Vector4[] CreateFileData(out float minx, out float miny, out float minz, out float maxx, out float maxy, out float maxz)
        {
            //string file = "../../test/data/1.2-with-color.las";
            string file = "../../test/data/hobu.las";

            Vector4 red = new Vector4(1, 0, 0, 1);
            Vector4 green = new Vector4(0, 1, 0, 1);
            Vector4 blue = new Vector4(0, 0, 1, 1);

            List<Vector4> points = new List<Vector4>();

            istream istr = Utils.openFile(file);
            m_reader = new LiblasReader(istr);

            Stage stage = m_reader;
            {
                int np = (int)m_reader.getNumPoints();
                if (np > 100000)
                {
                    int step = np / 100000;
                    m_filter = new DecimationFilter(m_reader, step);
                    stage = m_filter;
                }
            }

            Header header = stage.getHeader();
            Bounds_double bounds = header.getBounds();
            minx = (float)bounds.getMinimum(0);
            miny = (float)bounds.getMinimum(1);
            minz = (float)bounds.getMinimum(2);
            maxx = (float)bounds.getMaximum(0);
            maxy = (float)bounds.getMaximum(1);
            maxz = (float)bounds.getMaximum(2);

            // 1.2-with-color
            //minx 635619.9f
            //miny 848899.7f
            //minz 406.59f
            //maxx 638982.563
            //maxy 853535.438
            //maxz 586.38

            ulong numPoints = stage.getNumPoints();
            //numPoints = Math.Min(numPoints, 100000);

            Schema schema = stage.getHeader().getSchema();
            SchemaLayout layout = new SchemaLayout(schema);

            PointData data = new PointData(layout,(uint)numPoints);

            uint numRead = stage.read(data);

            uint offsetX = (uint)schema.getDimensionIndex(Dimension.Field.Field_X);
            uint offsetY = (uint)schema.getDimensionIndex(Dimension.Field.Field_Y);
            uint offsetZ = (uint)schema.getDimensionIndex(Dimension.Field.Field_Z);
            uint offsetR = (uint)schema.getDimensionIndex(Dimension.Field.Field_Red);
            uint offsetG = (uint)schema.getDimensionIndex(Dimension.Field.Field_Green);
            uint offsetBG = (uint)schema.getDimensionIndex(Dimension.Field.Field_Blue);

            for (uint index=0; index<numRead; index++)
            {   
                Int32 xraw = data.getField_Int32(index, offsetX);
                Int32 yraw = data.getField_Int32(index, offsetY);
                Int32 zraw = data.getField_Int32(index, offsetZ);
                float x = (float)schema.getDimension(offsetX).getNumericValue_Int32(xraw);
                float y = (float)schema.getDimension(offsetY).getNumericValue_Int32(yraw);
                float z = (float)schema.getDimension(offsetZ).getNumericValue_Int32(zraw);

                if (index == 0)
                {
                    minx = maxx = x;
                    miny = maxy = y;
                    minz = maxz = z;
                }
                else
                {
                    minx = Math.Min(x, minx);
                    miny = Math.Min(y, miny);
                    minz = Math.Min(z, minz);
                    maxx = Math.Max(x, maxx);
                    maxy = Math.Max(y, maxy);
                    maxz = Math.Max(z, maxz);
                }

                UInt16 r = data.getField_UInt16(index, offsetX);
                UInt16 g = data.getField_UInt16(index, offsetY);
                UInt16 b = data.getField_UInt16(index, offsetZ);

                points.Add(new Vector4(x, y, z, 1));

                float rf = (float)r / 65535.0f;
                float gf = (float)g / 65535.0f;
                float bf = (float)b / 65535.0f;
                points.Add(new Vector4(rf, gf, bf, 1));
                //points.Add(blue);
            }

            Utils.closeFile(istr);

            return points.ToArray();
        }

        private Vector4[] CreateRandomData(out float minx, out float miny, out float minz, out float maxx, out float maxy, out float maxz)
        {
            Vector4 red = new Vector4(1, 0, 0, 1);
            Vector4 green = new Vector4(0, 1, 0, 1);
            Vector4 blue = new Vector4(0, 0, 1, 1);

            List<Vector4> points = new List<Vector4>();

            System.Random rand = new System.Random();

            for (int p = 0; p < 1000; p++)
            {
                float x = (float)rand.NextDouble() * 100;
                float y = (float)rand.NextDouble() * 10 + 45;
                float z = (float)rand.NextDouble() * 100;
                points.Add(new Vector4(x, y, z, 1));

                float r = (float)rand.NextDouble();
                float g = (float)rand.NextDouble();
                float b = (float)rand.NextDouble();
                points.Add(new Vector4(r, g, b, 1));
            }

            minx = 0;
            miny = 0;
            minz = 0;
            maxx = 100;
            maxy = 100;
            maxz = 100;

            return points.ToArray();
        }

        #region INotifyPropertyChanged
        public event PropertyChangedEventHandler PropertyChanged;

        private void NotifyPropertyChanged(string info)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(info));
            }
        }
        #endregion
    }
}
