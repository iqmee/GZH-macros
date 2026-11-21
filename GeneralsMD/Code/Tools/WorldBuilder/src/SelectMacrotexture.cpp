/*
**	Command & Conquer Generals Zero Hour(tm)
**	Copyright 2025 Electronic Arts Inc.
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

// SelectMacrotexture.cpp : implementation file
//

#include "StdAfx.h"
#include <set>
#include "WorldBuilder.h"
#include "WorldBuilderDoc.h"
#include "wbview3d.h"
#include "SelectMacrotexture.h"
#include "Common/Dict.h"
#include "Common/FileSystem.h"
#include "Common/GlobalData.h"
#include "Common/MapObject.h"
#include "Common/WellKnownKeys.h"
#include "W3DDevice/GameClient/HeightMap.h"

/////////////////////////////////////////////////////////////////////////////
// SelectMacrotexture dialog


SelectMacrotexture::SelectMacrotexture(CWnd* pParent /*=nullptr*/)
	: CDialog(SelectMacrotexture::IDD, pParent)
	, m_initialUseLightMap(FALSE)
	, m_initializing(FALSE)
{
	//{{AFX_DATA_INIT(SelectMacrotexture)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void SelectMacrotexture::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(SelectMacrotexture)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(SelectMacrotexture, CDialog)
	//{{AFX_MSG_MAP(SelectMacrotexture)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

#define DEFAULT "***Default"
#define MACROTEXTURE_PREFIX "TSNoiseUrb"

/////////////////////////////////////////////////////////////////////////////
// SelectMacrotexture message handlers

BOOL SelectMacrotexture::OnInitDialog()
{
	CDialog::OnInitDialog();

	m_initializing = TRUE;
	m_initialUseLightMap = TheGlobalData ? TheGlobalData->m_useLightMap : FALSE;

	Bool exists = FALSE;
	Dict *worldDict = MapObject::getWorldDict();
	if (worldDict != nullptr)
	{
		m_initialMacroTexture = worldDict->getAsciiString(TheKey_macroTexture, &exists);
		if (!exists)
		{
			m_initialMacroTexture.clear();
		}
	}
	else if (TheTerrainRenderObject != nullptr)
	{
		m_initialMacroTexture = TheTerrainRenderObject->getMacroTextureName();
	}
	m_selectedMacroTexture = m_initialMacroTexture;

	CWnd *pWnd = GetDlgItem(IDC_TEXTURE_TREEVIEW);
	CRect rect;
	pWnd->GetWindowRect(&rect);

	ScreenToClient(&rect);
	rect.DeflateRect(2,2,2,2);
	m_textureTreeView.Create(TVS_HASLINES|TVS_LINESATROOT|TVS_HASBUTTONS|
		TVS_SHOWSELALWAYS|TVS_DISABLEDRAGDROP, rect, this, IDC_TERRAIN_TREEVIEW);
	m_textureTreeView.ShowWindow(SW_SHOW);

	HTREEITEM hDefaultItem = nullptr;
	HTREEITEM hInitialSelection = nullptr;

	{
		TVINSERTSTRUCT ins;
		::memset(&ins, 0, sizeof(ins));
		ins.hParent = TVI_ROOT;
		ins.hInsertAfter = TVI_SORT;
		ins.item.mask = TVIF_PARAM|TVIF_TEXT;
		ins.item.lParam = -1;
		char defaultText[] = DEFAULT;
		ins.item.pszText = defaultText;
		ins.item.cchTextMax = strlen(defaultText);
		hDefaultItem = m_textureTreeView.InsertItem(&ins);
	}

	{
		char fileBuf[_MAX_PATH];
		FilenameList filenameList;
		TheFileSystem->getFileListInDirectory(TGA_DIR_PATH, "*", filenameList, FALSE);

		const Int dirLen = strlen(TGA_DIR_PATH);
		std::set<AsciiString> seenLowerNames;

		for (FilenameList::const_iterator it = filenameList.begin(); it != filenameList.end(); ++it)
		{
			const AsciiString& fullPath = *it;
			if (fullPath.getLength() <= dirLen || !fullPath.startsWithNoCase(TGA_DIR_PATH))
				continue;

			const char *relativeName = fullPath.str() + dirLen;
			while (*relativeName == '/' || *relativeName == '\\')
				++relativeName;

			// Ignore files in subdirectories of Art/Textures/.
			if (*relativeName == '\0' || strchr(relativeName, '/') != nullptr || strchr(relativeName, '\\') != nullptr)
				continue;

			AsciiString filename(relativeName);
			if (!filename.startsWithNoCase(MACROTEXTURE_PREFIX))
				continue;

			if (!filename.endsWithNoCase(".dds") && !filename.endsWithNoCase(".tga"))
				continue;

			AsciiString lowerName = filename;
			lowerName.toLower();
			if (seenLowerNames.find(lowerName) != seenLowerNames.end())
				continue;
			seenLowerNames.insert(lowerName);

			strlcpy(fileBuf, filename.str(), ARRAY_SIZE(fileBuf));
			TVINSERTSTRUCT ins;
			::memset(&ins, 0, sizeof(ins));
			ins.hParent = TVI_ROOT;
			ins.hInsertAfter = TVI_SORT;
			ins.item.mask = TVIF_PARAM|TVIF_TEXT;
			ins.item.lParam = -1;
			ins.item.pszText = fileBuf;
			ins.item.cchTextMax = strlen(fileBuf);
			HTREEITEM child = m_textureTreeView.InsertItem(&ins);

			if (!m_initialMacroTexture.isEmpty() && filename.compareNoCase(m_initialMacroTexture) == 0)
			{
				hInitialSelection = child;
			}
		}
	}

	if (m_initialMacroTexture.isEmpty())
	{
		m_textureTreeView.SelectItem(hDefaultItem);
	}
	else if (hInitialSelection != nullptr)
	{
		m_textureTreeView.SelectItem(hInitialSelection);
	}
	m_initializing = FALSE;

	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

BOOL SelectMacrotexture::OnNotify(WPARAM wParam, LPARAM lParam, LRESULT* pResult)
{
	if (!m_initializing && lParam != 0)
	{
		NMTREEVIEW *pHdr = (NMTREEVIEW *)lParam;
		if (pHdr->hdr.hwndFrom == m_textureTreeView.m_hWnd && pHdr->hdr.code == TVN_SELCHANGED)
		{
			char buffer[_MAX_PATH];
			HTREEITEM hItem = m_textureTreeView.GetSelectedItem();
			if (hItem != nullptr)
			{
				TVITEM item;
				::memset(&item, 0, sizeof(item));
				item.mask = TVIF_HANDLE|TVIF_PARAM|TVIF_TEXT|TVIF_STATE;
				item.hItem = hItem;
				item.pszText = buffer;
				item.cchTextMax = sizeof(buffer)-2;
				if (m_textureTreeView.GetItem(&item))
				{
					if (strcmp(buffer, DEFAULT) == 0)
					{
						m_selectedMacroTexture.clear();
					}
					else
					{
						m_selectedMacroTexture = AsciiString(buffer);
					}

					if (TheTerrainRenderObject != nullptr)
					{
						TheTerrainRenderObject->updateMacroTexture(m_selectedMacroTexture);
					}
					TheWritableGlobalData->m_useLightMap = true;

					WbView3d *p3View = CWorldBuilderDoc::GetActive3DView();
					if (p3View != nullptr)
					{
						p3View->redraw();
					}
				}
			}
		}
	}
	return CDialog::OnNotify(wParam, lParam, pResult);
}

void SelectMacrotexture::OnOK()
{
	Dict *worldDict = MapObject::getWorldDict();
	if (worldDict != nullptr)
	{
		if (m_selectedMacroTexture.isEmpty())
		{
			worldDict->remove(TheKey_macroTexture);
		}
		else
		{
			worldDict->setAsciiString(TheKey_macroTexture, m_selectedMacroTexture);
		}
	}

	if (TheTerrainRenderObject != nullptr)
	{
		TheTerrainRenderObject->updateMacroTexture(m_selectedMacroTexture);
	}

	if (m_selectedMacroTexture.compareNoCase(m_initialMacroTexture) != 0)
	{
		CWorldBuilderDoc *pDoc = CWorldBuilderDoc::GetActiveDoc();
		if (pDoc != nullptr)
		{
			pDoc->SetModifiedFlag();
		}
	}

	WbView3d *p3View = CWorldBuilderDoc::GetActive3DView();
	if (p3View != nullptr)
	{
		p3View->redraw();
	}

	CDialog::OnOK();
}

void SelectMacrotexture::OnCancel()
{
	if (TheTerrainRenderObject != nullptr)
	{
		TheTerrainRenderObject->updateMacroTexture(m_initialMacroTexture);
	}
	TheWritableGlobalData->m_useLightMap = m_initialUseLightMap;

	WbView3d *p3View = CWorldBuilderDoc::GetActive3DView();
	if (p3View != nullptr)
	{
		p3View->redraw();
	}

	CDialog::OnCancel();
}
