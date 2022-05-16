
from .CamCal_config import Config
import numpy as np

class CVanLnSel:

    def __init__(self):
        # background image for plotting results
        # configuration parameters
        # flag of completing the selection of vanishing lines
        self.m_bSelVanLnFlg = False
        self.nodes = []

    def process(self,FPts):
        voVanPt = []
        self.nodes=FPts
        if Config.CalSelVanLnFlg:
            voVanPt = self.compVanPts()
        else:
            voVanPt.append(Config.getCalVr())
            voVanPt.append(Config.getCalVl())
        return voVanPt

    def compVanPts(self):
        voVanPt=[]
        if len(self.nodes) != 8:
            print("Error: not enough nodes of vanishing lines selected")
        m_voNd=self.nodes
        fSlpPr1Ln1 = (m_voNd[0][1] - m_voNd[1][1]) / (m_voNd[0][0] - m_voNd[1][0])
        fSlpPr1Ln2 = (m_voNd[2][1] - m_voNd[3][1]) / (m_voNd[2][0] - m_voNd[3][0])
        oVanPt1=np.zeros((2,1))
        oVanPt1[0] = ((fSlpPr1Ln1 * m_voNd[0][0]) - (fSlpPr1Ln2 * m_voNd[2][0]) + m_voNd[2][1] - m_voNd[0][1]) / (fSlpPr1Ln1 - fSlpPr1Ln2)
        oVanPt1[1] = m_voNd[0][1] + ((oVanPt1[0] - m_voNd[0][0]) * fSlpPr1Ln1)

        fSlpPr2Ln1 = (m_voNd[4][1] - m_voNd[5][1]) / (m_voNd[4][0] - m_voNd[5][0])
        fSlpPr2Ln2 = (m_voNd[6][1] - m_voNd[7][1]) / (m_voNd[6][0] - m_voNd[7][0])
        oVanPt2=np.zeros((2,1))
        oVanPt2[0] = ((fSlpPr2Ln1 * m_voNd[4][0]) - (fSlpPr2Ln2 * m_voNd[6][0]) + m_voNd[6][1] - m_voNd[4][1]) / (fSlpPr2Ln1 - fSlpPr2Ln2)
        oVanPt2[1] = m_voNd[4][1] + ((oVanPt2[0] - m_voNd[4][0]) * fSlpPr2Ln1)

        if oVanPt1[0] >= oVanPt2[0]:
            voVanPt.append(oVanPt1)
            voVanPt.append(oVanPt2)
        else:
            voVanPt.append(oVanPt2)
            voVanPt.append(oVanPt1)

        return voVanPt

'''
    def estVyByCM(self, m_voVyCand):
        fSumX = 0
        fSumY = 0
        m_oVy = np.zeros((2, 1))
        nCandNum = len(m_voVyCand)
        for i in range(len(m_voVyCand)):
            fSumX += m_voVyCand[i, 0]
            fSumY += m_voVyCand[i, 1]
        m_oVy[0] = fSumX / nCandNum
        m_oVy[1] = fSumY / nCandNum
        return m_oVy

        # estimates vertical vanishing point Vy by RANSAC algorithm


    def estVyByRS(self,m_voVyCand):
        self.CamParam
        iRand0 = 0
        iRand1 = 0
        iRand2 = 0
        iRand3 = 0
        iIter = 0
        nInlCnt
        nMaxInlNum = Config.INT_MIN
        nCandNum = len(m_voVyCand.size)
        fDist2Vy = 0.0
        fVyCandX = 0.0
        fVyCandY = 0.0
        fMinVyCandX = DBL_MAX
        fMaxVyCandX = -DBL_MAX
        std::vector < bool > vbInl;
        std::vector < bool > vbMaxInl;
        while Config.VY_RS_ITER_NUM > iIter:
            std::vector < bool > ().swap(vbInl);
            for i in range(nCandNum):
                vbInl.push_back(true)
            nInlCnt = 0
            fMinVyCandX = DBL_MAX
            fMaxVyCandX = -DBL_MAX
            iRand0 = rand() % nCandNum
            iRand1 = rand() % nCandNum
            iRand2 = rand() % nCandNum
            iRand3 = rand() % nCandNum
            while (iRand0 == iRand1)
                iRand1 = rand() % nCandNum
            while ((iRand0 == iRand2) | | (iRand1 == iRand2))
                iRand2 = rand() % nCandNum;
            while ((iRand0 == iRand3) | | (iRand1 == iRand3) | | (iRand2 == iRand3))
                iRand3 = rand() % nCandNum;
            fVyCandX = (m_voVyCand[iRand0].x + m_voVyCand[iRand1].x + m_voVyCand[iRand2].x + m_voVyCand[
                iRand3].x) / 4.0
            fVyCandY = (m_voVyCand[iRand0][1] + m_voVyCand[iRand1].y + m_voVyCand[iRand2].y + m_voVyCand[
                iRand3].y) / 4.0

            for (int i = 0; i < nCandNum; i++)
                fDist2Vy = std::sqrt(((fVyCandY - m_voVyCand[i].y) * (fVyCandY - m_voVyCand[i].y)) + (
                        (fVyCandX - m_voVyCand[i].x) * (fVyCandX - m_voVyCand[i].x)))
                if Config.VY_RS_DIST_THLD * m_oCfg.getFrmSz().width > fDist2Vy:
                    if (fMinVyCandX > m_voVyCand[i].x):
                        fMinVyCandX = m_voVyCand[i].x
                    if (fMaxVyCandX < m_voVyCand[i].x):
                        fMaxVyCandX = m_voVyCand[i].x
                    nInlCnt += 1
                else:
                    vbInl[i] = false
        # add weight on nInlCnt based on the width of inlier points
        nInlCnt = (double)
        nInlCnt * ((double)m_oCfg.getFrmSz().width / (fMaxVyCandX - fMinVyCandX))
        if (nMaxInlNum < nInlCnt)
            nMaxInlNum = nInlCnt
            vbMaxInl = vbInl
        iIter += 1


    for (int i = nCandNum - 1; i >= 0; i--)
        if (!vbMaxInl[i])
            m_voVyCand.erase(m_voVyCand.begin() + i)
        # calculate the center of mass of the inliers as the final Vy
    estVyByCM()
    std::vector < bool > ().swap(vbInl)
    std::vector < bool > ().swap(vbMaxInl)
    return vbInl


    # estimates horizon line Linf by linear regression
    def estLinfByLR(self):
        fSumX = 0.0
        fSumY = 0.0
        nCandNum = self.m_voLinfCand.size()
        for (int i = 0; i < nCandNum; i++)
            fSumX += (double)
            m_voLinfCand[i].x
            fSumY += (double)
            m_voLinfCand[i].y

        fAveX = fSumX / (double)
        nCandNum
        fAveY = fSumY / (double)
        nCandNum
        fCovXY = 0.0
        fVarX = 0.0
        fVarY = 0.0
        for (int i = 0; i < nCandNum; i++)
            fCovXY += (m_voLinfCand[i].x - fAveX) * (m_voLinfCand[i].y - fAveY)
            fVarX += (m_voLinfCand[i].x - fAveX) * (m_voLinfCand[i].x - fAveX)
            fVarY += (m_voLinfCand[i].y - fAveY) * (m_voLinfCand[i].y - fAveY)
        m_fLinfSlp = fCovXY / fVarX;
        m_fLinfItcp = fAveY - (m_fLinfSlp * fAveX);
        m_fLinfCorr = fCovXY / (std::sqrt(fVarX) * std::sqrt(fVarY))
        return xxx


    # estimates horizon line Linf by RANSAC algorithm
    def estLinfByRS():
        int
        iRand0 = 0, iRand1 = 0, iIter = 0, nInlCnt, nMaxInlNum = INT_MIN, nCandNum = m_voLinfCand.size();


    double
    fDist2Linf = 0.0;
    std::vector < bool > vbInl;
    std::vector < bool > vbMaxInl;
    while (LINF_RS_ITER_NUM > iIter)
        std::vector < bool > ().swap(vbInl);
        for (int i = 0; i < nCandNum; i++)
            vbInl.push_back(true);
        nInlCnt = 0
        iRand0 = rand() % nCandNum;
        iRand1 = rand() % nCandNum;
        while (iRand0 == iRand1)
            iRand1 = rand() % nCandNum;
        double
        fDenom = std::sqrt(
            ((m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y) * (m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y))
            + ((m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x) * (
                    m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x)));
        for (int i = 0; i < nCandNum; i++)
            fDist2Linf = abs(((m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y) * m_voLinfCand[i].x)
                             - ((m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x) * m_voLinfCand[i].y)
                             + (m_voLinfCand[iRand1].x * m_voLinfCand[iRand0].y)
                             - (m_voLinfCand[iRand0].x * m_voLinfCand[iRand1].y)) / fDenom;
            if ((LINF_RS_DIST_THLD * (double)m_oCfg.getFrmSz().height) > fDist2Linf)
            nInlCnt + +
        else
            vbInl[i] = false
    if (nMaxInlNum < nInlCnt)
        nMaxInlNum = nInlCnt;
        vbMaxInl = vbInl;
    iIter + +;

    for (int i = nCandNum - 1; i >= 0; i--)
        if (!vbMaxInl[i])
            m_voLinfCand.erase(m_voLinfCand.begin() + i);

        # perform linear regression on the inliers to determine the final Linf
    estLinfByLR()
    std::vector < bool > ().swap(vbInl);
    std::vector < bool > ().swap(vbMaxInl);
    return xxxxx


# estimates principal point by assuming it as the image center

# estimates vanishing points Vr and Vl on the horizon line
    def estVrVl(self):
        # assume the line connecting Vr and Vy always goes through the bottom right corner of the frame image
        self.CamParam
        fAVrVy = (self.CamParam.img_height - 1) - m_oVy[1]
        fBVrVy = m_oVy.x - (m_oCfg.getFrmSz().width - 1)
        fCVrVy = ((m_oCfg.getFrmSz().width - 1) * m_oVy.y) - (m_oVy.x * (m_oCfg.getFrmSz().height - 1))
        fDVrVy = (fAVrVy * (-1.0f)) - (m_fLinfSlp * fBVrVy)
        if (0.0f == fDVrVy)
            int
            nBtmRgtXInc = 0;
            while (0.0f == fDVrVy)
                nBtmRgtXInc += 1;
                fAVrVy = (m_oCfg.getFrmSz().height - 1) - m_oVy.y;
                fBVrVy = m_oVy.x - (m_oCfg.getFrmSz().width + nBtmRgtXInc - 1);
                fCVrVy = ((m_oCfg.getFrmSz().width - 1) * m_oVy.y) - (m_oVy.x * (m_oCfg.getFrmSz().height - 1));
                fDVrVy = (fAVrVy * (-1.0f)) - (m_fLinfSlp * fBVrVy);

        m_oVr.x = ((fBVrVy * m_fLinfItcp) - ((-1.0f) * fCVrVy)) / fDVrVy;
        m_oVr.y = ((m_fLinfSlp * fCVrVy) - (fAVrVy * m_fLinfItcp)) / fDVrVy;
        float
        fVlPrinPtSlp = -1.0
        f / (-fAVrVy / fBVrVy);
        float
        fDVlPrinPt = fVlPrinPtSlp - m_fLinfSlp;
        m_oVl.x = ((fVlPrinPtSlp * m_oPrinPt.x) - m_oPrinPt.y + m_fLinfItcp) / fDVlPrinPt;
        m_oVl.y = ((fVlPrinPtSlp * m_fLinfItcp) + (m_fLinfSlp * fVlPrinPtSlp * m_oPrinPt.x) - (
                m_fLinfSlp * m_oPrinPt.y)) / fDVlPrinPt;
        while ((m_oCfg.getFrmSz().width > m_oVr.x) | | (0 <= m_oVl.x))
            m_oVr.x -= 1.0
            f;
            m_oVr.y = (m_oVr.x * m_fLinfSlp) + m_fLinfItcp;
            fAVrVy = m_oVr.y - m_oVy.y;
            fBVrVy = m_oVy.x - m_oVr.x;
            fVlPrinPtSlp = -1.0
            f / (-fAVrVy / fBVrVy);
            fDVlPrinPt = fVlPrinPtSlp - m_fLinfSlp;
            m_oVl.x = ((fVlPrinPtSlp * m_oPrinPt.x) - m_oPrinPt.y + m_fLinfItcp) / fDVlPrinPt;
            m_oVl.y = ((fVlPrinPtSlp * m_fLinfItcp) + (m_fLinfSlp * fVlPrinPtSlp * m_oPrinPt.x) - (
                    m_fLinfSlp * m_oPrinPt.y)) / fDVlPrinPt;
            if (m_oCfg.getFrmSz().width > m_oVr.x)
                break


'''
