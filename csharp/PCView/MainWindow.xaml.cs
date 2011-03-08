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
        private Point m_downPoint2D;
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

            //Vector4[] points = CreateFileData(out minx, out miny, out minz, out maxx, out maxy, out maxz);
            Vector4[] points = CreateRandomData(out minx, out miny, out minz, out maxx, out maxy, out maxz);
            
            m_renderEngine.SetPoints(points, minx, miny, minz, maxx, maxy, maxz);

            m_renderEngine.CameraPosition = new Vector3(minx, miny, minz);
            m_renderEngine.TargetPosition = new Vector3(maxx, maxy, maxz);

            m_trackball = new Trackball(450, 450);

            m_renderEngine.CameraPosition = new Vector3(0.5f, 0.5f, -5.1f);
            m_renderEngine.TargetPosition = new Vector3(0, 0, 0);

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
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                m_downPoint2D = e.GetPosition(x_contentControl);
                m_trackball.Previous = m_downPoint2D;
                m_isMoving = true;
            }
        }

        private void Mouse_MouseUp(object sender, MouseButtonEventArgs e)
        {
            m_downPoint2D = e.GetPosition(x_contentControl);
            m_trackball.Previous = m_downPoint2D;
            m_isMoving = false;
        }

        private void Mouse_MouseMove(object sender, MouseEventArgs e)
        {
            {
                Point mouse = e.GetPosition(x_contentControl);

                Vector3 world = m_renderEngine.ConvertToWorldCoordinates(mouse, (float)x_contentControl.ActualWidth, (float)x_contentControl.ActualHeight);

                PositionString = string.Format(
                    CultureInfo.InvariantCulture,
                    "[{0},{1}]  X={2}  Y={3}  Z={4}",
                    mouse.X,
                    mouse.Y,
                    world.X.ToString("F1", CultureInfo.InvariantCulture),
                    world.Y.ToString("F1", CultureInfo.InvariantCulture),
                    world.Z.ToString("F1", CultureInfo.InvariantCulture));
            }

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                if (m_isMoving)
                {
                    Point currPoint2D = e.GetPosition(x_contentControl);
                    Matrix rotation = m_trackball.Update(currPoint2D);

                    m_renderEngine.Rotation = rotation;
                }
            }

            return;
        }

        private void Mouse_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            Vector3 camera = m_renderEngine.CameraPosition;
            camera.Z += e.Delta / 100.0f;
            camera.Z += e.Delta / 100.0f;
            m_renderEngine.CameraPosition = camera;
        }
        #endregion

        private Vector4[] CreateFileData(out float minx, out float miny, out float minz, out float maxx, out float maxy, out float maxz)
        {
            string file = "../../test/data/1.2-with-color.las";

            Vector4 red = new Vector4(1, 0, 0, 1);
            Vector4 green = new Vector4(0, 1, 0, 1);
            Vector4 blue = new Vector4(0, 0, 1, 1);

            List<Vector4> points = new List<Vector4>();

            istream istr = Utils.openFile(file);
            LiblasReader reader = new LiblasReader(istr);

            Header header = reader.getHeader();
            Bounds_double bounds = header.getBounds();
            minx = (float)bounds.getMinimum(0);
            miny = (float)bounds.getMinimum(1);
            minz = (float)bounds.getMinimum(2);
            maxx = (float)bounds.getMaximum(0);
            maxy = (float)bounds.getMaximum(1);
            maxz = (float)bounds.getMaximum(2);

            minx -= 635619.9f;
            miny -= 848899.7f;
            minz -= 406.59f;
            maxx -= 635619.9f;
            maxy -= 848899.7f;
            maxz -= 406.59f;


            ulong numPoints = reader.getNumPoints();

            Schema schema = reader.getHeader().getSchema();
            SchemaLayout layout = new SchemaLayout(schema);

            PointData data = new PointData(layout, 1000);

            uint numRead = reader.read(data);

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

                UInt16 r = data.getField_UInt16(index, offsetX);
                UInt16 g = data.getField_UInt16(index, offsetY);
                UInt16 b = data.getField_UInt16(index, offsetZ);

                x -= 635619.9f;
                y -= 848899.7f;
                z -= 406.59f;

                points.Add(new Vector4(x, y, z, 1));

                float rf = (float)r / 65535.0f;
                float gf = (float)g / 65535.0f;
                float bf = (float)b / 65535.0f;
                //points.Add(new Vector4(rf, gf, bf, 1));
                points.Add(blue);
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
