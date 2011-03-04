//-----------------------------------------------------------------------
// <copyright file="MouseRenderEngine.cs" company="Flaxen Geo">
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
    using System;
    using System.Windows;
    using SlimDX;
    using SlimDX.D3DCompiler;
    using SlimDX.Direct3D10;
    using SlimDX.DXGI;
    using Buffer = SlimDX.Direct3D10.Buffer;
    using System.Collections.Generic;

    /// <summary>
    /// This is an example rendering engine which demonstrates some basic mouse actions such as
    /// camera movement and mouse coordinate conversion.  Code is provided to demonstrate explicit
    /// handling of the world/view/perspective transforms.
    /// </summary>
    public class PointCloudRenderEngine : SimpleRenderEngine
    {
        private bool m_perspective = true;

        private DataStream m_sampleStream;
        private InputLayout m_sampleLayout;
        private Buffer m_sampleVertices;
        
        private DataStream m_boxStream;
        private InputLayout m_boxLayout;
        private Buffer m_boxVertices;

        private Effect m_effect;

        private int m_pointSize = 32;  // 4*4 + 4*4
        private Vector4[] m_samplePoints;
        private Vector4[] m_boxPoints;

        private Matrix m_projectionTransform;
        private Matrix m_worldTransform;
        private Matrix view;
        private Vector3 m_worldUp;

        /// <summary>
        /// Initializes a new instance of the MouseRenderEngine class.
        /// </summary>
        /// <param name="points">an array of points to be displayed</param>
        public PointCloudRenderEngine()
        {
            CameraPosition = new Vector3(0.1f, 0.1f, -9.1f);
            TargetPosition = new Vector3(0, 0, 0);
        }


        private Vector4[] CreateBox(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
        {
            float f = 0.3f;
            Vector4 red = new Vector4(f, 0, 0, 1);
            Vector4 green= new Vector4(0, f, 0, 1);
            Vector4 blue  = new Vector4(0, 0, f, 1);

            List<Vector4> points = new List<Vector4>();

            // front square
            {
                points.Add(new Vector4(xmin, ymin, zmin, 1));
                points.Add(red);

                points.Add(new Vector4(xmax, ymin, zmin, 1));
                points.Add(red);

                points.Add(new Vector4(xmax, ymin, zmin, 1));
                points.Add(green);

                points.Add(new Vector4(xmax, ymax, zmin, 1));
                points.Add(green);

                points.Add(new Vector4(xmax, ymax, zmin, 1));
                points.Add(red);

                points.Add(new Vector4(xmin, ymax, zmin, 1));
                points.Add(red);

                points.Add(new Vector4(xmin, ymax, zmin, 1));
                points.Add(green);

                points.Add(new Vector4(xmin, ymin, zmin, 1));
                points.Add(green);
            }

            // back square
            {
                points.Add(new Vector4(xmin, ymin, zmax, 1));
                points.Add(red);

                points.Add(new Vector4(xmax, ymin, zmax, 1));
                points.Add(red);

                points.Add(new Vector4(xmax, ymin, zmax, 1));
                points.Add(green);

                points.Add(new Vector4(xmax, ymax, zmax, 1));
                points.Add(green);

                points.Add(new Vector4(xmax, ymax, zmax, 1));
                points.Add(red);

                points.Add(new Vector4(xmin, ymax, zmax, 1));
                points.Add(red);

                points.Add(new Vector4(xmin, ymax, zmax, 1));
                points.Add(green);

                points.Add(new Vector4(xmin, ymin, zmax, 1));
                points.Add(green);
            }

            // z-connectors
            {
                points.Add(new Vector4(xmin, ymin, zmin, 1));
                points.Add(blue);

                points.Add(new Vector4(xmin, ymin, zmax, 1));
                points.Add(blue);

                points.Add(new Vector4(xmin, ymax, zmin, 1));
                points.Add(blue);

                points.Add(new Vector4(xmin, ymax, zmax, 1));
                points.Add(blue);

                points.Add(new Vector4(xmax, ymin, zmin, 1));
                points.Add(blue);

                points.Add(new Vector4(xmax, ymin, zmax, 1));
                points.Add(blue);

                points.Add(new Vector4(xmax, ymax, zmin, 1));
                points.Add(blue);

                points.Add(new Vector4(xmax, ymax, zmax, 1));
                points.Add(blue);
            }

            return points.ToArray();
        }

        /// <summary>
        /// Gets or sets the camera's postion.  The values are in world coordinates.
        /// </summary>
        public Vector3 CameraPosition { get; set; }

        /// <summary>
        /// Gets or sets the camera target's postion.  The values are in world coordinates.
        /// </summary>
        public Vector3 TargetPosition { get; set; }


        /// <summary>
        /// Implements the logic to render the points.  Called by the SlimDXControl object.
        /// </summary>
        public override void Render()
        {
            Device.OutputMerger.SetTargets(SampleDepthView, SampleRenderView);
            Device.Rasterizer.SetViewports(new Viewport(0, 0, WindowWidth, WindowHeight, 0.0f, 1.0f));

            Device.ClearDepthStencilView(SampleDepthView, DepthStencilClearFlags.Depth | DepthStencilClearFlags.Stencil, 1.0f, 0);

            Device.ClearRenderTargetView(SampleRenderView, new SlimDX.Color4(1.0f, 0.0f, 0.0f, 0.0f));

            // we may have changed our camera or target positions, so rerun the transforms
            SetTransforms();

            {
                Device.InputAssembler.SetInputLayout(m_sampleLayout);
                Device.InputAssembler.SetPrimitiveTopology(PrimitiveTopology.PointList);
                Device.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(m_sampleVertices, m_pointSize, 0));

                EffectTechnique technique = m_effect.GetTechniqueByIndex(0);
                EffectPass pass = technique.GetPassByIndex(0);

                for (int i = 0; i < technique.Description.PassCount; ++i)
                {
                    pass.Apply();
                    Device.Draw(m_samplePoints.Length, 0);
                }
            }

            Device.Flush();

            {
                Device.InputAssembler.SetInputLayout(m_boxLayout);
                Device.InputAssembler.SetPrimitiveTopology(PrimitiveTopology.LineList);
                Device.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(m_boxVertices, m_pointSize, 0));

                EffectTechnique technique = m_effect.GetTechniqueByIndex(0);
                EffectPass pass = technique.GetPassByIndex(0);

                for (int i = 0; i < technique.Description.PassCount; ++i)
                {
                    pass.Apply();
                    Device.Draw(m_boxPoints.Length, 0);
                }
            }

            Device.Flush();
        }

        /// <summary>
        /// Compute the position of the mouse in world coordinates.
        /// </summary>
        /// <param name="mouse">mouse position, in screen space</param>
        /// <param name="width">width of window (the content control), in screen space</param>
        /// <param name="height">height of window (the content control), in screen space</param>
        /// <returns>calculated mouse position, in world coordinates</returns>
        public Vector3 ConvertToWorldCoordinates(Point mouse, float width, float height)
        {
            float mouseX = (float)mouse.X;
            float mouseY = (float)mouse.Y;

            float vx = (+2.0f * mouseX / width) - 1.0f;
            float vy = (-2.0f * mouseY / height) + 1.0f;

            vx = vx / m_projectionTransform.M11;
            vy = vy / m_projectionTransform.M22;

            // in view (camera) space
            Vector3 rayOrigin = new Vector3(0, 0, 0); // where camera is
            Vector3 rayDir = new Vector3(vx, vy, 1.0f);

            Matrix viewInv = view;
            viewInv.Invert();

            //// view space to world space
            rayOrigin = Vector3.TransformCoordinate(rayOrigin, viewInv);
            rayDir = Vector3.TransformNormal(rayDir, viewInv);

            Matrix worldInv = m_worldTransform;
            worldInv.Invert();

            //// world space to local space
            rayOrigin = Vector3.TransformCoordinate(rayOrigin, worldInv);
            rayDir = Vector3.TransformNormal(rayDir, worldInv);

            float c = -rayOrigin.Z / rayDir.Z;

            float worldX = rayOrigin.X + (c * rayDir.X);
            float worldY = rayOrigin.Y + (c * rayDir.Y);
            float worldZ = rayOrigin.Z + (c * rayDir.Z);

            return new Vector3(worldX, worldY, worldZ);
        }

        /// <summary>
        /// Implements the creation of the effect and the sets up the vertices.
        /// </summary>
        /// <param name="control">the associated SlimDXControl object</param>
        public override void Initialize(SlimDXControl control)
        {
            base.Initialize(control);

            try
            {
                m_effect = Effect.FromFile(Device, "pipeline.fx", "fx_4_0", ShaderFlags.None, EffectFlags.None, null, null);
            }
            catch (SlimDX.CompilationException)
            {
                // handy spot for a breakpoint to catch shader syntax errors
                throw;
            }

            Device.Flush();

            return;
        }

        public void SetPoints(Vector4[] points)
        {
            m_samplePoints = points;

            float xmin = m_samplePoints[0].X;
            float xmax = m_samplePoints[0].X;
            float ymin = m_samplePoints[0].Y;
            float ymax = m_samplePoints[0].Y;
            float zmin = m_samplePoints[0].Z;
            float zmax = m_samplePoints[0].Z;

            for (int i=2; i<m_samplePoints.Length; i+=2)
            {
                xmin = Math.Min(xmin, m_samplePoints[i].X);
                xmax = Math.Max(xmax, m_samplePoints[i].X);
                ymin = Math.Min(ymin, m_samplePoints[i].Y);
                ymax = Math.Max(ymax, m_samplePoints[i].Y);
                zmin = Math.Min(zmin, m_samplePoints[i].Z);
                zmax = Math.Max(zmax, m_samplePoints[i].Z);
            }

            m_boxPoints = CreateBox(xmin, xmax, ymin, ymax, zmin, zmax);

            SetupSamples();
            SetupBox();

            Device.Flush();

            return;
        }

        private void SetupSamples()
        {
            EffectTechnique technique = m_effect.GetTechniqueByIndex(0);
            EffectPass pass = technique.GetPassByIndex(0);

            InputElement[] inputElements = new[] 
                {
                    new InputElement("POSITION", 0, Format.R32G32B32A32_Float, 0, 0),
                    new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0) 
                };
            m_sampleLayout = new InputLayout(Device, pass.Description.Signature, inputElements);

            m_sampleStream = new DataStream(m_samplePoints.Length * m_pointSize, true, true);

            m_sampleStream.WriteRange(m_samplePoints);
            m_sampleStream.Position = 0;

            BufferDescription bufferDesc = new BufferDescription()
            {
                BindFlags = BindFlags.VertexBuffer,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = m_samplePoints.Length * m_pointSize,
                Usage = ResourceUsage.Default
            };
            m_sampleVertices = new Buffer(Device, m_sampleStream, bufferDesc);
        }

        private void SetupBox()
        {
            EffectTechnique technique = m_effect.GetTechniqueByIndex(0);
            EffectPass pass = technique.GetPassByIndex(0);

            InputElement[] inputElements = new[] 
                {
                    new InputElement("POSITION", 0, Format.R32G32B32A32_Float, 0, 0),
                    new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0) 
                };
            m_boxLayout = new InputLayout(Device, pass.Description.Signature, inputElements);

            m_boxStream = new DataStream(m_boxPoints.Length * m_pointSize, true, true);

            m_boxStream.WriteRange(m_boxPoints);
            m_boxStream.Position = 0;

            BufferDescription bufferDesc = new BufferDescription()
            {
                BindFlags = BindFlags.VertexBuffer,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = m_boxPoints.Length * m_pointSize,
                Usage = ResourceUsage.Default
            };
            m_boxVertices = new Buffer(Device, m_boxStream, bufferDesc);
        }

        /// <summary>
        /// Disposes of our managed resources.
        /// </summary>
        protected override void DisposeManaged()
        {
            {
                if (m_sampleVertices != null)
                {
                    m_sampleVertices.Dispose();
                    m_sampleVertices = null;
                }

                if (m_sampleLayout != null)
                {
                    m_sampleLayout.Dispose();
                    m_sampleLayout = null;
                }

                if (m_sampleStream != null)
                {
                    m_sampleStream.Dispose();
                    m_sampleStream = null;
                }
            }

            {
                if (m_boxVertices != null)
                {
                    m_boxVertices.Dispose();
                    m_boxVertices = null;
                }

                if (m_boxLayout != null)
                {
                    m_boxLayout.Dispose();
                    m_boxLayout = null;
                }

                if (m_boxStream != null)
                {
                    m_boxStream.Dispose();
                    m_boxStream = null;
                }
            }

            if (m_effect != null)
            {
                m_effect.Dispose();
                m_effect = null;
            }

            if (SharedTexture != null)
            {
                SharedTexture.Dispose();
                SharedTexture = null;
            }

            base.DisposeManaged();

            return;
        }

        /// <summary>
        /// Sets up our canonical transform matrices.
        /// </summary>
        private void SetTransforms()
        {
            // world transfrom: from local coordinates to world coordinates
            // in our case, from the range [0..100] to [0..1]
            m_worldTransform = Matrix.Scaling(0.01f, 0.01f, 0.01f);

            // view transform: from world coordinates to view (camera, eye) coordinates
            // the "up" direction is the Y axis
            m_worldUp = new Vector3(0, 1, 0);
            view = Matrix.LookAtLH(CameraPosition, TargetPosition, m_worldUp);

            // projection transform: from view coordinates to perspective space
            float znear = 0.0f; // in view space
            float zfar = 10.0f; // in view space
            if (m_perspective)
            {
                float fovY = (float)(Math.PI * 0.25); // radians, 45 deg
                float aspect = 1.0f;
                m_projectionTransform = Matrix.PerspectiveFovLH(fovY, aspect, znear, zfar);
            }
            else
            {
                float width = 2.0f;
                float height = 2.0f;
                m_projectionTransform = Matrix.OrthoLH(width, height, znear, zfar);
            }

            // compute the summary transform, and push it into the gpu
            SlimDX.Matrix wvp = m_worldTransform * view * m_projectionTransform;
            EffectMatrixVariable wvpTransform = m_effect.GetVariableByName("gWVP").AsMatrix();
            wvpTransform.SetMatrix(wvp);

            return;
        }
    }
}
