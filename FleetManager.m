classdef FleetManager< matlab.apps.AppBase

    % --- ARAYÜZ BİLEŞENLERİ ---
    properties (Access = public)
        UIFigure                    matlab.ui.Figure
        
        % Paneller
        LeftPanel                   matlab.ui.container.Panel
        TopInfoPanel                matlab.ui.container.Panel
        CenterMapPanel              matlab.ui.container.Panel
        RightGraphPanel             matlab.ui.container.Panel
        
        % Grafikler
        UIAxes_Map                  matlab.ui.control.UIAxes
        UIAxes_Vel                  matlab.ui.control.UIAxes
        UIAxes_Wheel                matlab.ui.control.UIAxes
        
        % --- SOL MENÜ KONTROLLERİ ---
        MapGroupPanel               matlab.ui.container.Panel
        MapTypeLabel                matlab.ui.control.Label
        MapTypeDropDown             matlab.ui.control.DropDown
        GenerateMapButton           matlab.ui.control.Button
        
        RobotGroupPanel             matlab.ui.container.Panel
        RobotCountLabel             matlab.ui.control.Label
        RobotcountEditField         matlab.ui.control.NumericEditField
        PlaceRobotsButton           matlab.ui.control.Button
        
        PlanGroupPanel              matlab.ui.container.Panel
        PlannerLabel                matlab.ui.control.Label
        PlannerDropDown             matlab.ui.control.DropDown
        ConflictLabel               matlab.ui.control.Label
        ConflictDropDown            matlab.ui.control.DropDown
        StartSimulationButton       matlab.ui.control.Button
        
        ViewGroupPanel              matlab.ui.container.Panel
        ShowPathCheckBox            matlab.ui.control.CheckBox
        ShowTrailCheckBox           matlab.ui.control.CheckBox
        ShowWaitForCheckBox         matlab.ui.control.CheckBox
        
        StatusLabel                 matlab.ui.control.Label
        
        % --- SAĞ PANEL KONTROLLERİ (YENİ) ---
        GraphFilterLabel            matlab.ui.control.Label
        GraphFilterDropDown         matlab.ui.control.DropDown
        
        % --- BİLGİ ETİKETLERİ ---
        TimeValLabel                matlab.ui.control.Label
        CollisionValLabel           matlab.ui.control.Label
        FinishedValLabel            matlab.ui.control.Label
        
        LegendPanel                 matlab.ui.container.Panel
    end
    
    properties (Access = private)
        mapGrid;
        mapObj;
        robots;
        isSimRunning = false;
        
        % Grafik Nesneleri
        animLinesV;
        animLinesVL;
        animLinesVR;
        arrowHandles;
        
        % Renkler (Dark Mode)
        bgColor = [0.12 0.14 0.18];
        panelColor = [0.16 0.18 0.24];
        axesColor = [0.1 0.1 0.12];
        textColor = [0.9 0.9 0.9];
        accentColor = [0.2 0.6 1.0];
        successColor = [0.2 0.8 0.2];
        warningColor = [1.0 0.6 0.0];
        
        L = 0.5; % Robot Aks Açıklığı
    end
    
    methods (Access = private)
        
        % --- 1. HARİTA OLUŞTURMA ---
        function createMap(app, mode)
            gridSize = 50;
            gridMap = zeros(gridSize, gridSize);
            
            % Dış Duvarlar
            gridMap(1, :) = 1; gridMap(end, :) = 1;
            gridMap(:, 1) = 1; gridMap(:, end) = 1;
            
            if strcmp(mode, 'Narrow Corridor')
                % DAR KORİDOR (Deadlock Garantili)
                gridMap(:, 22:24) = 1; % Sol duvar
                gridMap(:, 27:29) = 1; % Sağ duvar
                gridMap(20:30, 22:29) = 0; % Orta Geçiş
                gridMap(5:10, 22:29) = 0;  % Alt Geçiş
                gridMap(15, 1:22) = 1;
                gridMap(35, 29:50) = 1;
            else
                % RANDOM (Karmaşık Şekiller)
                numObs = 12; 
                for k = 1:numObs
                    r = randi([5, 40]); c = randi([5, 40]);
                    type = rand();
                    try
                        if type < 0.4 % Blok
                            gridMap(r:r+4, c:c+4) = 1; 
                        elseif type < 0.7 % L-Şekli
                            gridMap(r:r+6, c:c+2) = 1;
                            gridMap(r+4:r+6, c:c+6) = 1;
                        else % Duvar
                            gridMap(r, c:c+8) = 1;
                        end
                    catch; end
                end
            end
            
            % Harita Temizliği
            invMap = ~gridMap; 
            CC = bwconncomp(invMap);
            numPixels = cellfun(@numel, CC.PixelIdxList);
            [~, idx] = max(numPixels);
            validMap = ones(size(gridMap));
            validMap(CC.PixelIdxList{idx}) = 0;
            
            app.mapGrid = validMap;
            app.mapObj = binaryOccupancyMap(validMap, 1);
        end
        
        % --- 2. HEDEF ATAMA ---
        function [s, g] = assignPointsWithClass(app, distClass)
            mat = getOccupancy(app.mapObj);
            mapDiag = sqrt(50^2 + 50^2);
            [rows, cols] = size(mat);
            
            validPair = false; counter = 0;
            s = [5, 5]; g = [10, 10];
            
            while ~validPair && counter < 2000
                counter = counter + 1;
                s_cand = [randi([3, 47]), randi([3, 47])];
                g_cand = [randi([3, 47]), randi([3, 47])];
                
                % Güvenlik Kontrolü
                try
                    s_safe = sum(sum(mat(s_cand(1)-1:s_cand(1)+1, s_cand(2)-1:s_cand(2)+1))) == 0;
                    g_safe = sum(sum(mat(g_cand(1)-1:g_cand(1)+1, g_cand(2)-1:g_cand(2)+1))) == 0;

                    if s_safe && g_safe
                        d = norm(s_cand - g_cand);
                        ratio = d / mapDiag;
                        
                        isDistOK = false;
                        if distClass == 1 && (ratio >= 0.1 && ratio < 0.3), isDistOK = true;
                        elseif distClass == 2 && (ratio >= 0.3 && ratio < 0.6), isDistOK = true;
                        elseif distClass == 3 && (ratio >= 0.6), isDistOK = true;
                        end
                        
                        if isDistOK
                            s = [s_cand(2), s_cand(1)]; g = [g_cand(2), g_cand(1)];
                            validPair = true;
                        end
                    end
                catch; end
            end
            
            % Buldozer
            try
               yS = s(2); xS = s(1); yG = g(2); xG = g(1);
               setOccupancy(app.mapObj, [yS-1 yS+1; xS-1 xS+1], 0); 
               setOccupancy(app.mapObj, [yG-1 yG+1; xG-1 xG+1], 0);
            catch; end
        end
        
        % --- 3. PLANLAYICI (FAIL-SAFE) ---
        function path = runPlanner(app, startPos, goalPos)
            plannerType = app.PlannerDropDown.Value;
            path = [];
            mapCopy = copy(app.mapObj);
            
            if checkOccupancy(mapCopy, startPos) == 1, startPos = startPos + [1 1]; end
            if checkOccupancy(mapCopy, goalPos) == 1, goalPos = goalPos + [1 1]; end
            
            try
                switch plannerType
                    case 'A* (Optimal)'
                        astar = plannerAStarGrid(mapCopy);
                        startGrid = world2grid(mapCopy, startPos);
                        goalGrid = world2grid(mapCopy, goalPos);
                        pathGrid = plan(astar, startGrid, goalGrid);
                        path = grid2world(mapCopy, pathGrid);
                        
                    case 'Dijkstra (Garantili)'
                        dijkstra = plannerAStarGrid(mapCopy); 
                        startGrid = world2grid(mapCopy, startPos);
                        goalGrid = world2grid(mapCopy, goalPos);
                        pathGrid = plan(dijkstra, startGrid, goalGrid);
                        path = grid2world(mapCopy, pathGrid);
                        
                    case 'PRM (Probabilistic)'
                        prm = mobileRobotPRM(mapCopy);
                        prm.NumNodes = 2000; prm.ConnectionDistance = 30; prm.InflationRadius = 0.05; 
                        path = findpath(prm, startPos, goalPos);
                end
                
                % FAIL-SAFE: Yol yoksa A* dene
                if isempty(path)
                     astar = plannerAStarGrid(mapCopy);
                     try
                        startGrid = world2grid(mapCopy, startPos);
                        goalGrid = world2grid(mapCopy, goalPos);
                        pathGrid = plan(astar, startGrid, goalGrid);
                        path = grid2world(mapCopy, pathGrid);
                     catch; end
                end
            catch; path = []; end
        end
        
        % --- GRAFİK TEMİZLEME VE HAZIRLAMA ---
        function clearGraphs(app)
             cla(app.UIAxes_Vel); cla(app.UIAxes_Wheel);
             app.animLinesV = []; app.animLinesVL = []; app.animLinesVR = [];
             app.arrowHandles = [];
             
             hold(app.UIAxes_Vel, 'on'); grid(app.UIAxes_Vel, 'on');
             title(app.UIAxes_Vel, 'Doğrusal Hız (v)', 'Color', app.textColor);
             
             hold(app.UIAxes_Wheel, 'on'); grid(app.UIAxes_Wheel, 'on');
             title(app.UIAxes_Wheel, 'Tekerlek Hızları', 'Color', app.textColor);
        end
        
        % --- YENİ: GRAFİK GÖRÜNÜRLÜĞÜNÜ GÜNCELLE ---
        function updateGraphVisibility(app)
            selected = app.GraphFilterDropDown.Value;
            
            % Eğer robotlar henüz oluşmadıysa çık
            if isempty(app.animLinesV), return; end
            
            for i = 1:length(app.animLinesV)
                % Görünürlük Durumu
                if strcmp(selected, 'Tümü') || strcmp(selected, ['Robot ' num2str(i)])
                    state = 'on';
                else
                    state = 'off';
                end
                
                % Tüm çizgilere uygula
                app.animLinesV(i).Visible = state;
                app.animLinesVL(i).Visible = state;
                app.animLinesVR(i).Visible = state;
            end
            
            % Başlığı güncelle
            if strcmp(selected, 'Tümü')
                title(app.UIAxes_Vel, 'Doğrusal Hız (Tümü)', 'Color', app.textColor);
            else
                title(app.UIAxes_Vel, ['Doğrusal Hız (' selected ')'], 'Color', app.textColor);
            end
        end
    end
    
    % --- BUTON AKSİYONLARI ---
    methods (Access = private)
        
        function GenerateMapButtonPushed(app, event)
            app.createMap(app.MapTypeDropDown.Value);
            cla(app.UIAxes_Map);
            show(app.mapObj, 'Parent', app.UIAxes_Map);
            title(app.UIAxes_Map, 'Binary Occupancy Grid', 'Color', app.textColor);
            axis(app.UIAxes_Map, [0 50 0 50]); 
            app.UIAxes_Map.Color = app.axesColor;
            app.UIAxes_Map.XColor = app.textColor; app.UIAxes_Map.YColor = app.textColor;
            app.UIAxes_Map.GridColor = [0.3 0.3 0.3];
            
            app.robots = [];
            app.clearGraphs();
            app.GraphFilterDropDown.Items = {'-'}; % Sıfırla
            app.TimeValLabel.Text = "0.0s"; app.CollisionValLabel.Text = "0"; app.FinishedValLabel.Text = "0/0";
            app.StatusLabel.Text = "Harita Hazır.";
        end
        
        function PlaceRobotsButtonPushed(app, event)
            if isempty(app.mapObj), uialert(app.UIFigure, "Önce Harita!", "Hata"); return; end
            
            n = app.RobotcountEditField.Value;
            app.robots = struct('id', {}, 'start', {}, 'goal', {}, 'path', {}, 'color', {}, 'pos', {}, 'status', {}, 'theta', {});
            
            app.clearGraphs();
            cla(app.UIAxes_Map);
            show(app.mapObj, 'Parent', app.UIAxes_Map);
            hold(app.UIAxes_Map, 'on');
            axis(app.UIAxes_Map, [0 50 0 50]);
            
            colors = hsv(n); colors = colors + 0.2; colors(colors>1) = 1; 
            
            % Filtre Menüsünü Doldur
            filterItems = {'Tümü'};
            for i=1:n, filterItems{end+1} = ['Robot ' num2str(i)]; end
            app.GraphFilterDropDown.Items = filterItems;
            app.GraphFilterDropDown.Value = 'Robot 1'; % Varsayılan Odak
            
            for i = 1:n
                randVal = rand();
                if randVal < 0.3, dClass = 1; elseif randVal < 0.7, dClass = 2; else, dClass = 3; end
                [s, g] = app.assignPointsWithClass(dClass);
                
                app.robots(i).id = i; app.robots(i).start = s; app.robots(i).goal = g;
                app.robots(i).pos = s; app.robots(i).theta = 0;
                app.robots(i).color = colors(i,:); app.robots(i).path = []; app.robots(i).status = 'Ready';
                
                plot(app.UIAxes_Map, s(1), s(2), 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
                plot(app.UIAxes_Map, g(1), g(2), 's', 'Color', colors(i,:), 'LineWidth', 3, 'MarkerSize', 12, 'MarkerFaceColor', colors(i,:)*0.3);
                
                app.animLinesV = [app.animLinesV; animatedline(app.UIAxes_Vel, 'Color', colors(i,:), 'LineWidth', 2)];
                app.animLinesVL = [app.animLinesVL; animatedline(app.UIAxes_Wheel, 'Color', colors(i,:), 'LineWidth', 1.5)];
                app.animLinesVR = [app.animLinesVR; animatedline(app.UIAxes_Wheel, 'Color', colors(i,:), 'LineWidth', 1.5, 'LineStyle', '--')];
            end
            
            hold(app.UIAxes_Map, 'off');
            app.updateGraphVisibility(); % Görünürlüğü ayarla
            
            app.FinishedValLabel.Text = ["0/" num2str(n)];
            app.StatusLabel.Text = "Robotlar Yerleştirildi.";
        end
        
        function GraphFilterChanged(app, event)
            app.updateGraphVisibility();
        end
        
        function StartSimulationButtonPushed(app, event)
            if isempty(app.robots), return; end
            app.StatusLabel.Text = "Rotalar Hesaplanıyor..."; 
            app.StartSimulationButton.BackgroundColor = app.warningColor;
            app.StartSimulationButton.Text = "HESAPLANIYOR...";
            drawnow;
            
            validCount = 0;
            for i = 1:length(app.robots)
                path = app.runPlanner(app.robots(i).start, app.robots(i).goal);
                
                if ~isempty(path)
                    app.robots(i).path = path; app.robots(i).pathIndex = 1;
                    app.robots(i).status = 'Active';
                    validCount = validCount + 1;
                    
                    if app.ShowPathCheckBox.Value
                        hold(app.UIAxes_Map, 'on');
                        plot(app.UIAxes_Map, path(:,1), path(:,2), '--', 'Color', [app.robots(i).color 0.5], 'LineWidth', 1.5);
                        hold(app.UIAxes_Map, 'off');
                    end
                else
                    app.robots(i).status = 'NoPath';
                end
            end
            
            if validCount == 0
                app.StartSimulationButton.Text = "BAŞLAT";
                app.StartSimulationButton.BackgroundColor = app.successColor;
                app.StatusLabel.Text = "Yol Bulunamadı.";
                uialert(app.UIFigure, "Hiçbir robot için yol bulunamadı! Haritayı yenileyin.", "Hata");
                return;
            end
            
            app.StatusLabel.Text = "Simülasyon Çalışıyor...";
            app.StartSimulationButton.BackgroundColor = app.successColor;
            app.StartSimulationButton.Text = "ÇALIŞIYOR";
            
            app.isSimRunning = true;
            t = 0; dt = 0.2; colCount = 0; finishedCount = 0;
            
            while app.isSimRunning
                t = t + dt;
                app.TimeValLabel.Text = [num2str(t, '%.1f') 's'];
                
                if ~isempty(app.arrowHandles), delete(app.arrowHandles); app.arrowHandles = []; end
                anyMove = false;
                finishedReal = 0;
                
                for i = 1:length(app.robots)
                    % Bitmiş veya yolu olmayanları kontrol et
                    if strcmp(app.robots(i).status, 'Finished')
                        finishedReal = finishedReal + 1; continue; 
                    elseif strcmp(app.robots(i).status, 'NoPath')
                        % Yolu olmayanlar sayaca dahil değil
                        continue;
                    end
                    
                    if isempty(app.robots(i).path) || app.robots(i).pathIndex >= size(app.robots(i).path, 1)
                        distToGoal = norm(app.robots(i).pos - app.robots(i).goal);
                        if distToGoal < 1.0
                            app.robots(i).status = 'Finished';
                            finishedReal = finishedReal + 1;
                        end
                        continue;
                    end
                    
                    anyMove = true;
                    
                    curr = app.robots(i).pos; 
                    idx = app.robots(i).pathIndex;
                    target = app.robots(i).path(idx + 1, :); 
                    dist = norm(target - curr);
                    
                    % --- ÇAKIŞMA ---
                    wait = false; waitingForWho = -1;
                    if strcmp(app.ConflictDropDown.Value, 'Bekle')
                        for j = 1:length(app.robots)
                            if i ~= j && strcmp(app.robots(j).status, 'Active')
                                if norm(app.robots(j).pos - curr) < 2.5
                                    if i > j 
                                        wait = true; waitingForWho = j; 
                                        colCount = colCount + 0.05; 
                                        app.CollisionValLabel.Text = num2str(floor(colCount)); 
                                    end
                                end
                            end
                        end
                    end
                    
                    if wait && app.ShowWaitForCheckBox.Value && waitingForWho ~= -1
                        hold(app.UIAxes_Map, 'on');
                        h = quiver(app.UIAxes_Map, curr(1), curr(2), ...
                            app.robots(waitingForWho).pos(1)-curr(1), ...
                            app.robots(waitingForWho).pos(2)-curr(2), ...
                            0, 'Color', 'r', 'LineWidth', 2);
                        app.arrowHandles = [app.arrowHandles, h]; hold(app.UIAxes_Map, 'off');
                    end
                    
                    % Hareket
                    if wait
                        v = 0; w = 0; vL = 0; vR = 0;
                    else
                        step = 1.0; 
                        if dist < step
                            app.robots(i).pos = target; app.robots(i).pathIndex = idx + 1; v = dist/dt;
                        else
                            dir = (target - curr) / dist; app.robots(i).pos = curr + dir * step; v = step/dt; 
                        end
                        
                        targetTheta = atan2(target(2)-curr(2), target(1)-curr(1));
                        dTheta = angdiff(app.robots(i).theta, targetTheta); 
                        w = dTheta / dt; 
                        app.robots(i).theta = targetTheta;
                        vR = v + (w * app.L / 2); vL = v - (w * app.L / 2);
                        
                        hold(app.UIAxes_Map, 'on');
                        plot(app.UIAxes_Map, app.robots(i).pos(1), app.robots(i).pos(2), 'o', 'MarkerFaceColor', app.robots(i).color, 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
                        if app.ShowTrailCheckBox.Value
                            plot(app.UIAxes_Map, app.robots(i).pos(1), app.robots(i).pos(2), '.', 'Color', app.robots(i).color, 'MarkerSize', 4);
                        end
                        hold(app.UIAxes_Map, 'off');
                    end
                    
                    addpoints(app.animLinesV(i), t, v);
                    addpoints(app.animLinesVL(i), t, vL);
                    addpoints(app.animLinesVR(i), t, vR);
                end
                
                app.FinishedValLabel.Text = [num2str(finishedReal) '/' num2str(length(app.robots))];
                axis(app.UIAxes_Vel, 'tight'); axis(app.UIAxes_Wheel, 'tight');
                
                if ~anyMove || finishedReal == length(app.robots)
                    app.isSimRunning = false; 
                    app.StatusLabel.Text = "Tamamlandı.";
                    app.StartSimulationButton.Text = "BAŞLAT";
                    uialert(app.UIFigure, "Simülasyon Bitti.", "Tamam"); 
                end
                drawnow limitrate; pause(0.02);
            end
        end
    end
    
    % --- UI KURULUM (Colors Local) ---
    methods (Access = public)
        function app = FleetManager
            createComponents(app)
            registerApp(app, app.UIFigure)
            if nargout == 0, clear app, end
        end
        function delete(app), delete(app.UIFigure), end
    end
    
    methods (Access = private)
        function createComponents(app)
            % Renkler (Local)
            bgColor = [0.12 0.14 0.18]; panelColor = [0.16 0.18 0.24];
            axesColor = [0.1 0.1 0.12]; textColor = [0.9 0.9 0.9];
            accentColor = [0.2 0.6 1.0]; buttonColor = [0.25 0.3 0.4];
            successColor = [0.2 0.8 0.2]; warningColor = [1.0 0.6 0.0];
            
            app.UIFigure = uifigure('Visible', 'off', 'Position', [50 50 1400 800], 'Name', 'FLEET MANAGER v7.0 ', 'Color', bgColor);
            
            % --- SOL PANEL ---
            app.LeftPanel = uipanel(app.UIFigure, 'Position', [10 10 300 780], 'BackgroundColor', panelColor, 'BorderType', 'none');
            uilabel(app.LeftPanel, 'Text', 'FLEET MANAGER', 'Position', [20 740 260 30], 'FontWeight', 'bold', 'FontSize', 22, 'FontColor', accentColor, 'HorizontalAlignment', 'center');
            
            % Harita Grubu
            app.MapGroupPanel = uipanel(app.LeftPanel, 'Title', 'HARİTA & ROBOTLAR', 'Position', [10 520 280 200], 'BackgroundColor', panelColor, 'ForegroundColor', textColor, 'FontSize', 12, 'FontWeight', 'bold');
            uilabel(app.MapGroupPanel, 'Text', 'Harita Tipi:', 'Position', [10 150 100 22], 'FontColor', textColor);
            app.MapTypeDropDown = uidropdown(app.MapGroupPanel, 'Items', {'Random', 'Narrow Corridor'}, 'Value', 'Random', 'Position', [10 130 260 22], 'BackgroundColor', buttonColor, 'FontColor', textColor);
            app.GenerateMapButton = uibutton(app.MapGroupPanel, 'Text', 'Harita Oluştur', 'Position', [10 90 260 30], 'BackgroundColor', accentColor, 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', createCallbackFcn(app, @GenerateMapButtonPushed, true));
            uilabel(app.MapGroupPanel, 'Text', 'Robot Sayısı:', 'Position', [10 50 100 22], 'FontColor', textColor);
            app.RobotcountEditField = uieditfield(app.MapGroupPanel, 'numeric', 'Value', 5, 'Position', [110 50 50 22], 'BackgroundColor', buttonColor, 'FontColor', textColor);
            app.PlaceRobotsButton = uibutton(app.MapGroupPanel, 'Text', 'Robotları Diz', 'Position', [10 10 260 30], 'BackgroundColor', accentColor, 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', createCallbackFcn(app, @PlaceRobotsButtonPushed, true));
            
            % Planlama Grubu
            app.PlanGroupPanel = uipanel(app.LeftPanel, 'Title', 'PLANLAMA & KONTROL', 'Position', [10 300 280 200], 'BackgroundColor', panelColor, 'ForegroundColor', textColor, 'FontSize', 12, 'FontWeight', 'bold');
            uilabel(app.PlanGroupPanel, 'Text', 'Planlayıcı:', 'Position', [10 150 100 22], 'FontColor', textColor);
            app.PlannerDropDown = uidropdown(app.PlanGroupPanel, 'Items', {'A* (Optimal)', 'Dijkstra (Garantili)', 'PRM (Probabilistic)'}, 'Position', [10 130 260 22], 'BackgroundColor', buttonColor, 'FontColor', textColor);
            uilabel(app.PlanGroupPanel, 'Text', 'Çatışma Modu:', 'Position', [10 90 100 22], 'FontColor', textColor);
            app.ConflictDropDown = uidropdown(app.PlanGroupPanel, 'Items', {'Bekle', 'Yok Say'}, 'Position', [10 70 260 22], 'BackgroundColor', buttonColor, 'FontColor', textColor);
            app.StartSimulationButton = uibutton(app.PlanGroupPanel, 'Text', 'SİMÜLASYONU BAŞLAT', 'Position', [10 10 260 40], 'BackgroundColor', successColor, 'FontColor', 'w', 'FontSize', 14, 'FontWeight', 'bold', 'ButtonPushedFcn', createCallbackFcn(app, @StartSimulationButtonPushed, true));
            
            % Görünüm
            app.ViewGroupPanel = uipanel(app.LeftPanel, 'Title', 'GÖRÜNÜM', 'Position', [10 120 280 160], 'BackgroundColor', panelColor, 'ForegroundColor', textColor, 'FontSize', 12, 'FontWeight', 'bold');
            app.ShowPathCheckBox = uicheckbox(app.ViewGroupPanel, 'Text', 'Planlanan Rota', 'Position', [10 110 150 22], 'Value', true, 'FontColor', textColor);
            app.ShowTrailCheckBox = uicheckbox(app.ViewGroupPanel, 'Text', 'Gerçekleşen İz', 'Position', [10 80 150 22], 'Value', true, 'FontColor', textColor);
            app.ShowWaitForCheckBox = uicheckbox(app.ViewGroupPanel, 'Text', 'Wait-For Graph', 'Position', [10 50 150 22], 'Value', true, 'FontColor', textColor);
            app.StatusLabel = uilabel(app.LeftPanel, 'Text', 'Bekleniyor...', 'Position', [20 20 260 22], 'FontColor', [0.7 0.7 0.7]);

            % --- ÜST PANEL ---
            app.TopInfoPanel = uipanel(app.UIFigure, 'Position', [320 680 1070 110], 'BackgroundColor', bgColor, 'BorderType', 'none');
            p1 = uipanel(app.TopInfoPanel, 'Position', [0 10 200 90], 'BackgroundColor', panelColor, 'BorderType', 'none');
            uilabel(p1, 'Text', 'SÜRE', 'Position', [10 60 180 20], 'FontColor', textColor, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            app.TimeValLabel = uilabel(p1, 'Text', '0.0s', 'Position', [10 20 180 40], 'FontColor', accentColor, 'FontSize', 24, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            
            p2 = uipanel(app.TopInfoPanel, 'Position', [220 10 200 90], 'BackgroundColor', panelColor, 'BorderType', 'none');
            uilabel(p2, 'Text', 'DEADLOCK / ÇARPIŞMA', 'Position', [10 60 180 20], 'FontColor', textColor, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            app.CollisionValLabel = uilabel(p2, 'Text', '0', 'Position', [10 20 180 40], 'FontColor', warningColor, 'FontSize', 24, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            
            p3 = uipanel(app.TopInfoPanel, 'Position', [440 10 200 90], 'BackgroundColor', panelColor, 'BorderType', 'none');
            uilabel(p3, 'Text', 'TAMAMLANAN', 'Position', [10 60 180 20], 'FontColor', textColor, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            app.FinishedValLabel = uilabel(p3, 'Text', '0/0', 'Position', [10 20 180 40], 'FontColor', successColor, 'FontSize', 24, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
            
            app.LegendPanel = uipanel(app.TopInfoPanel, 'Title', 'LEJANT', 'Position', [660 10 300 90], 'BackgroundColor', panelColor, 'ForegroundColor', textColor);
            uipanel(app.LegendPanel, 'Position', [10 40 20 20], 'BackgroundColor', 'w'); uilabel(app.LegendPanel, 'Text', 'Engel', 'Position', [35 40 50 20], 'FontColor', textColor);
            uipanel(app.LegendPanel, 'Position', [90 40 20 20], 'BackgroundColor', accentColor); uilabel(app.LegendPanel, 'Text', 'Robot', 'Position', [115 40 50 20], 'FontColor', textColor);
            uipanel(app.LegendPanel, 'Position', [170 40 20 20], 'BackgroundColor', [0.5 0.5 0.5]); uilabel(app.LegendPanel, 'Text', 'Hedef', 'Position', [195 40 50 20], 'FontColor', textColor);
            uipanel(app.LegendPanel, 'Position', [10 10 20 20], 'BackgroundColor', 'r'); uilabel(app.LegendPanel, 'Text', 'Wait-For', 'Position', [35 10 100 20], 'FontColor', textColor);

            % --- ORTA PANEL ---
            app.CenterMapPanel = uipanel(app.UIFigure, 'Position', [320 10 700 660], 'BackgroundColor', panelColor, 'BorderType', 'none');
            app.UIAxes_Map = uiaxes(app.CenterMapPanel, 'Position', [10 40 680 600]);
            title(app.UIAxes_Map, 'Binary Occupancy Grid', 'Color', textColor);
            app.UIAxes_Map.BackgroundColor = [0.1 0.1 0.12];
            app.UIAxes_Map.XColor = textColor; app.UIAxes_Map.YColor = textColor; app.UIAxes_Map.GridColor = [0.3 0.3 0.3];
            grid(app.UIAxes_Map, 'on');

            % --- SAĞ PANEL (GELİŞMİŞ GRAFİKLER) ---
            app.RightGraphPanel = uipanel(app.UIFigure, 'Position', [1030 10 360 660], 'BackgroundColor', panelColor, 'BorderType', 'none');
            uilabel(app.RightGraphPanel, 'Text', 'GERÇEK ZAMANLI VERİLER', 'Position', [10 630 340 25], 'FontColor', textColor, 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            
            % Grafik Filtre Menüsü
            app.GraphFilterLabel = uilabel(app.RightGraphPanel, 'Text', 'Grafik Odak:', 'Position', [10 600 80 22], 'FontColor', textColor);
            app.GraphFilterDropDown = uidropdown(app.RightGraphPanel, 'Items', {'-'}, 'Position', [90 600 250 22], 'BackgroundColor', buttonColor, 'FontColor', textColor, 'ValueChangedFcn', createCallbackFcn(app, @GraphFilterChanged, true));
            
            app.UIAxes_Vel = uiaxes(app.RightGraphPanel, 'Position', [10 330 340 260]);
            title(app.UIAxes_Vel, 'Doğrusal Hız (v)', 'Color', textColor);
            app.UIAxes_Vel.BackgroundColor = [0.1 0.1 0.12];
            app.UIAxes_Vel.XColor = textColor; app.UIAxes_Vel.YColor = textColor; app.UIAxes_Vel.GridColor = [0.3 0.3 0.3]; grid(app.UIAxes_Vel, 'on');
            
            app.UIAxes_Wheel = uiaxes(app.RightGraphPanel, 'Position', [10 10 340 260]);
            title(app.UIAxes_Wheel, 'Tekerlek Hızları', 'Color', textColor);
            app.UIAxes_Wheel.BackgroundColor = [0.1 0.1 0.12];
            app.UIAxes_Wheel.XColor = textColor; app.UIAxes_Wheel.YColor = textColor; app.UIAxes_Wheel.GridColor = [0.3 0.3 0.3]; grid(app.UIAxes_Wheel, 'on');

            app.UIFigure.Visible = 'on';
        end
    end
end