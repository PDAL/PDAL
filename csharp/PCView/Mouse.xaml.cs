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
    
    /// <summary>
    /// Implements the MouseExample test.
    /// </summary>
    public partial class Mouse : Window, INotifyPropertyChanged
    {
        private MouseRenderEngine m_renderEngine;
        private Point m_downPoint;
        private bool m_isMoving = false;
        private string m_positionString;

        /// <summary>
        /// Initializes a new instance of the Mouse class.
        /// </summary>
        public Mouse()
        {
            InitializeComponent();

            Vector4[] points = CreateAxes();
            m_renderEngine = new MouseRenderEngine(points);
            x_contentControl.RegisterRenderer(m_renderEngine);

            this.MouseMove += new MouseEventHandler(Mouse_MouseMove);
            this.MouseDown += new MouseButtonEventHandler(Mouse_MouseDown);
            this.MouseUp += new MouseButtonEventHandler(Mouse_MouseUp);
            this.MouseWheel += new MouseWheelEventHandler(Mouse_MouseWheel);
            
            PositionString = string.Empty;

            this.DataContext = this;

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
                m_downPoint = e.GetPosition(x_contentControl);
                m_isMoving = true;
            }
        }

        private void Mouse_MouseUp(object sender, MouseButtonEventArgs e)
        {
            m_isMoving = false;
        }

        private void Mouse_MouseMove(object sender, MouseEventArgs e)
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

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                if (m_isMoving)
                {
                    Point currPoint = e.GetPosition(x_contentControl);
                    Vector3 camera = m_renderEngine.CameraPosition;
                    Vector3 target = m_renderEngine.TargetPosition;
                    camera.X += (float)(m_downPoint.X - currPoint.X) / 100.0f;
                    target.X += (float)(m_downPoint.X - currPoint.X) / 100.0f;
                    camera.Y += -(float)(m_downPoint.Y - currPoint.Y) / 100.0f;
                    target.Y += -(float)(m_downPoint.Y - currPoint.Y) / 100.0f;
                    m_renderEngine.CameraPosition = camera;
                    m_renderEngine.TargetPosition = target;
                    m_downPoint = currPoint;
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

        private Vector4[] CreateAxes()
        {
            Vector4 red = new Vector4(1, 0, 0, 1);
            Vector4 green = new Vector4(0, 1, 0, 1);
            Vector4 blue = new Vector4(0, 0, 1, 1);

            List<Vector4> points = new List<Vector4>();

            // points on a line 0..100
            for (int p = 0; p < 100; p++)
            {
                points.Add(new Vector4(p, 0, 0, 1));
                points.Add(red);

                points.Add(new Vector4(0, p, 0, 1));
                points.Add(green);

                points.Add(new Vector4(0, 0, p, 1));
                points.Add(blue);
            }

            // points on mark at end of line
            for (int p = 0; p < 10; p++)
            {
                points.Add(new Vector4(100, p, 0, 1));
                points.Add(red);

                points.Add(new Vector4(p, 100, 0, 1));
                points.Add(green);

                points.Add(new Vector4(p, 0, 100, 1));
                points.Add(blue);
            }

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
