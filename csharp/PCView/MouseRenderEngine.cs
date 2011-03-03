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

    /// <summary>
    /// This is an example rendering engine which demonstrates some basic mouse actions such as
    /// camera movement and mouse coordinate conversion.  Code is provided to demonstrate explicit
    /// handling of the world/view/perspective transforms.
    /// </summary>
    public class MouseRenderEngine : SimpleRenderEngine
    {
        private bool m_perspective = true;
        private DataStream m_sampleStream;
        private InputLayout m_sampleLayout;
        private Buffer m_sampleVertices;
        private Effect m_sampleEffect;

        private int m_pointSize = 32;  // 4*4 + 4*4
        private Vector4[] m_points;

        private Matrix m_projectionTransform;
        private Matrix m_worldTransform;
        private Matrix view;
        private Vector3 m_worldUp;

        /// <summary>
        /// Initializes a new instance of the MouseRenderEngine class.
        /// </summary>
        /// <param name="points">an array of points to be displayed</param>
        public MouseRenderEngine(Vector4[] points)
        {
            CameraPosition = new Vector3(0.1f, 0.1f, -9.1f);
            TargetPosition = new Vector3(0, 0, 0);

            m_points = points;
        }

        /// <summary>
        /// Gets or sets the camera's postion.  The values are in world coordinates.
        /// </summary>
        public Vector3 CameraPosition { get; set; }

        /// <summary>
        /// Gets or sets the camera target's postion.  The values are in world coordinates.
        /// </summary>
        public Vector3 TargetPosition { get; set; }

        private int NumPoints
        {
            get
            {
                return m_points.Length;
            }
        }

        /// <summary>
        /// Implements the logic to render the points.  Called by the SlimDXControl object.
        /// </summary>
        public override void Render()
        {
            Device.OutputMerger.SetTargets(SampleDepthView, SampleRenderView);
            Device.Rasterizer.SetViewports(new Viewport(0, 0, WindowWidth, WindowHeight, 0.0f, 1.0f));

            Device.ClearDepthStencilView(SampleDepthView, DepthStencilClearFlags.Depth | DepthStencilClearFlags.Stencil, 1.0f, 0);

            Device.ClearRenderTargetView(SampleRenderView, new SlimDX.Color4(1.0f, 0.0f, 0.0f, 0.0f));

            Device.InputAssembler.SetInputLayout(m_sampleLayout);
            Device.InputAssembler.SetPrimitiveTopology(PrimitiveTopology.PointList);
            Device.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(m_sampleVertices, m_pointSize, 0));

            EffectTechnique technique = m_sampleEffect.GetTechniqueByIndex(0);
            EffectPass pass = technique.GetPassByIndex(0);

            // we may have changed our camera or target positions, so rerun the transforms
            SetTransforms();

            for (int i = 0; i < technique.Description.PassCount; ++i)
            {
                pass.Apply();
                Device.Draw(NumPoints, 0);
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
                m_sampleEffect = Effect.FromFile(Device, "Mouse.fx", "fx_4_0", ShaderFlags.None, EffectFlags.None, null, null);
            }
            catch (SlimDX.CompilationException)
            {
                // handy spot for a breakpoint to catch shader syntax errors
                throw;
            }

            EffectTechnique technique = m_sampleEffect.GetTechniqueByIndex(0);
            EffectPass pass = technique.GetPassByIndex(0);

            InputElement[] inputElements = new[] 
                {
                    new InputElement("POSITION", 0, Format.R32G32B32A32_Float, 0, 0),
                    new InputElement("COLOR", 0, Format.R32G32B32A32_Float, 16, 0) 
                };
            m_sampleLayout = new InputLayout(Device, pass.Description.Signature, inputElements);

            m_sampleStream = new DataStream(NumPoints * m_pointSize, true, true);

            m_sampleStream.WriteRange(m_points);
            m_sampleStream.Position = 0;

            BufferDescription bufferDesc = new BufferDescription()
            {
                BindFlags = BindFlags.VertexBuffer,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = NumPoints * m_pointSize,
                Usage = ResourceUsage.Default
            };
            m_sampleVertices = new Buffer(Device, m_sampleStream, bufferDesc);
            Device.Flush();

            return;
        }

        /// <summary>
        /// Disposes of our managed resources.
        /// </summary>
        protected override void DisposeManaged()
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

            if (m_sampleEffect != null)
            {
                m_sampleEffect.Dispose();
                m_sampleEffect = null;
            }

            if (m_sampleStream != null)
            {
                m_sampleStream.Dispose();
                m_sampleStream = null;
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
            EffectMatrixVariable wvpTransform = m_sampleEffect.GetVariableByName("gWVP").AsMatrix();
            wvpTransform.SetMatrix(wvp);

            return;
        }
    }
}
