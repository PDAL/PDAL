cbuffer cbPerObject
{
    float4x4 gWVP;
}

// VertexShader
void VS(float4 inputPos: POSITION, 
        float4 inputCol: COLOR,
        out float4 outputPos : SV_POSITION,
        out float4 outputCol : COLOR)
{
    //PS_IN output = (PS_IN)0;
    
    outputPos = mul(inputPos, gWVP);

    outputCol = inputCol;
}

// PixelShader
void PS(float4 inputPos: SV_POSITION, 
        float4 inputCol: COLOR,
        out float4 outputCol: SV_TARGET)
{
    outputCol = inputCol;
}

technique10 Render
{
    pass P0
    {
        SetGeometryShader( 0 );
        SetVertexShader( CompileShader( vs_4_0, VS() ) );
        SetPixelShader( CompileShader( ps_4_0, PS() ) );
    }
}
