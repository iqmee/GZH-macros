/*
**	Command & Conquer Generals Zero Hour(tm)
**	Copyright 2026 TheSuperHackers
**
**	This program is free software: you can redistribute it and/or modify
**	it under the terms of the GNU General Public License as published by
**	the Free Software Foundation, either version 3 of the License, or
**	(at your option) any later version.
**
**	This program is distributed in the hope that it will be useful,
**	but WITHOUT ANY WARRANTY; without even the implied warranty of
**	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**	GNU General Public License for more details.
**
**	You should have received a copy of the GNU General Public License
**	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <gtest/gtest.h>
#include <windows.h>
#include <cstdio>

#include "WW3D2/ww3d.h"
#include "WW3D2/dx8wrapper.h"
#include "WW3D2/rddesc.h"

namespace
{
HWND CreateTestGameWindow()
{
	WNDCLASS wc = {};
	wc.lpfnWndProc = DefWindowProc;
	wc.hInstance = GetModuleHandle(nullptr);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszClassName = TEXT("GZHResolutionTestWindow");
	RegisterClass(&wc);

	return CreateWindow(
		TEXT("GZHResolutionTestWindow"),
		TEXT("GZH Resolution Test"),
		WS_POPUP | WS_VISIBLE,
		0, 0, 800, 600,
		nullptr, nullptr, wc.hInstance, nullptr);
}
} // namespace

TEST(ResolutionFallbackTest, EnumeratesVirtualAspectResolutionsAndSortsAscending)
{
	HWND hwnd = CreateTestGameWindow();
	ASSERT_NE(hwnd, nullptr);

	if (WW3D::Init(hwnd) != WW3D_ERROR_OK || WW3D::Get_Render_Device_Count() <= 0)
	{
		DestroyWindow(hwnd);
		GTEST_SKIP() << "No Direct3D8 adapter available in this environment";
	}

	const RenderDeviceDescClass& desc = WW3D::Get_Render_Device_Desc(0);
	const DynamicVectorClass<ResolutionDescClass>& resList = desc.Enumerate_Resolutions();
	ASSERT_GT(resList.Count(), 0);

	MONITORINFO mi = { sizeof(MONITORINFO) };
	GetMonitorInfo(MonitorFromWindow(hwnd, MONITOR_DEFAULTTOPRIMARY), &mi);
	const int monW = mi.rcMonitor.right - mi.rcMonitor.left;
	const int monH = mi.rcMonitor.bottom - mi.rcMonitor.top;

	bool has1920x1440 = false;
	bool has2560x1440 = false;
	for (int i = 0; i < resList.Count(); ++i)
	{
		if (resList[i].Width == 1920 && resList[i].Height == 1440 && resList[i].BitDepth == 32)
			has1920x1440 = true;
		if (resList[i].Width == 2560 && resList[i].Height == 1440 && resList[i].BitDepth == 32)
			has2560x1440 = true;
	}
	if (monW >= 1920 && monH >= 1440)
	{
		EXPECT_TRUE(has1920x1440) << "1920x1440 must be available for >= 1440p displays even when omitted by driver";
	}
	if (monW >= 2560 && monH >= 1440)
	{
		EXPECT_TRUE(has2560x1440) << "2560x1440 must be available for >= 2560x1440 displays";
	}

	// Verify list is strictly sorted ascending by (Width, Height, BitDepth)
	for (int i = 1; i < resList.Count(); ++i)
	{
		const auto& prev = resList[i - 1];
		const auto& cur = resList[i];
		EXPECT_TRUE(
			prev.Width < cur.Width ||
			(prev.Width == cur.Width && prev.Height < cur.Height) ||
			(prev.Width == cur.Width && prev.Height == cur.Height && prev.BitDepth <= cur.BitDepth));
	}

	// Live hardware D3D8 device test (runs when D3DDEVTYPE_HAL is accessible on the desktop session)
	D3DCAPS8 halCaps = {};
	if (DX8Wrapper::_Get_D3D8() && SUCCEEDED(DX8Wrapper::_Get_D3D8()->GetDeviceCaps(0, D3DDEVTYPE_HAL, &halCaps)))
	{
		WW3D::Set_MSAA_Mode(WW3D::MULTISAMPLE_MODE_2X);
		ASSERT_EQ(WW3D::Set_Render_Device(0, 1920, 1440, 32, 0 /*fullscreen*/, true), WW3D_ERROR_OK);

		int curW = 0, curH = 0, curBits = 0;
		bool curWindowed = true;
		WW3D::Get_Device_Resolution(curW, curH, curBits, curWindowed);
		EXPECT_EQ(curW, 1920);
		EXPECT_EQ(curH, 1440);
		EXPECT_FALSE(curWindowed);

		DX8Wrapper::Begin_Scene();
		DX8Wrapper::Clear(true, true, Vector3(0.0f, 0.0f, 0.0f));
		DX8Wrapper::End_Scene(true);

		EXPECT_EQ(WW3D::Set_Device_Resolution(1600, 1200, 32, 0, true), WW3D_ERROR_OK);
		EXPECT_EQ(WW3D::Set_Device_Resolution(1920, 1440, 32, 0, true), WW3D_ERROR_OK);
		DX8Wrapper::On_App_Activate(false);
		DX8Wrapper::On_App_Activate(true);
	}

	WW3D::Shutdown();
	DestroyWindow(hwnd);
}
