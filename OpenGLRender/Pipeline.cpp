/*

	Copyright 2014 Rudy Snow

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pipeline.h"

const Matrix4f& Pipeline::GetVPTrans()
{
    Matrix4f CameraTranslationTrans, CameraRotateTrans, PersProjTrans;

    CameraTranslationTrans.InitTranslationTransform(-m_camera.Pos.x, -m_camera.Pos.y, -m_camera.Pos.z);
    CameraRotateTrans.InitCameraTransform(m_camera.Target, m_camera.Up);
    PersProjTrans.InitPersProjTransform(m_persProjInfo);
    
    m_VPTtransformation = PersProjTrans * CameraRotateTrans * CameraTranslationTrans;
    return m_VPTtransformation;
}

const Matrix4f& Pipeline::GetWorldTrans()
{
    Matrix4f ScaleTrans, RotateTrans, TranslationTrans;

    ScaleTrans.InitScaleTransform(m_scale.x, m_scale.y, m_scale.z);
    RotateTrans.InitRotateTransform(m_rotateInfo.x, m_rotateInfo.y, m_rotateInfo.z);
    if(quatRot) Quat2Matrix(rotation, RotateTrans);
    TranslationTrans.InitTranslationTransform(m_worldPos.x, m_worldPos.y, m_worldPos.z);

    m_WorldTransformation = TranslationTrans * RotateTrans * ScaleTrans;
    if(matRot) m_WorldTransformation = m_TRTransformation * ScaleTrans;
    return m_WorldTransformation;
}

const Matrix4f& Pipeline::GetWVPTrans()
{
    GetWorldTrans();
    GetVPTrans();

    m_WVPtransformation = m_VPTtransformation * m_WorldTransformation;
    return m_WVPtransformation;
}