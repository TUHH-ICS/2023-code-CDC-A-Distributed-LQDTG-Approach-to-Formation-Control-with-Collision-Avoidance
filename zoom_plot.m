function [zoom_utils] = zoom_plot(axes2zoom,options)
%zoom_plot Create a zoomed in axes object of axes2zoom
%   [zoom_utils] = zoom_plot(axes2zoom,options)
%   axes2zoom...the axes that are the source for the zoom
%   options...optional Name-Value pairs for the axes, rectangle, or arrows. Specify the options like:
%       options.axes.Names = {'Position','XLim'};
%       options.axes.Values = {[.6 .6 .3 .3],[1,3]};
%       options.rectangle.Names = {};
%       options.rectangle.Values = {};
%       options.arrows.Names = {'HeadLength','HeadWidth'};
%       options.arrows.Values = {8,8};
%
%   zoom_utils...the handles to all the created objects
%
%   Zoom-tool is selected by standard. ButtonDownFcn of axes moves the axes around.
%   Using zoom, pan, or clicking on the axes redraws the objects.
%
% Author: Zauner Michael
% Date: 10/05/2021
% create default options if not provided
if ~exist('options','var')
    options.axes.Names = {'Position'};
    options.axes.Values = {[.5 .5 .3 .3]};
    options.rectangle.Names = {};
    options.rectangle.Values = {};
    options.arrows.Names = {};
    options.arrows.Values = {};
end
% if we get options passed, make sure all the non-provided fields get initialized
if ~isfield(options,'axes')
    options.axes.Names = {};
    options.axes.Values = {};
end
if ~isfield(options,'rectangle')
    options.rectangle.Names = {};
    options.rectangle.Values = {};
end
if ~isfield(options,'arrows')
    options.arrows.Names = {};
    options.arrows.Values = {};
end
% Get parent Figure
fig = axes2zoom.Parent;
% make a copy and move it to the center;
zoomAxes = copyobj(axes2zoom,fig);
set(zoomAxes,options.axes.Names,options.axes.Values);
%% rectangle and arrow-lines
% make the rectangle in the original plot
rec = rectangle(axes2zoom,'Position',[zoomAxes.XLim(1), zoomAxes.YLim(1), diff(zoomAxes.XLim), diff(zoomAxes.YLim)]);
set(rec,options.rectangle.Names,options.rectangle.Values);
% make the annotation arrows
ano(1) = annotation('arrow',[0,0],[0,0]); % lower left
ano(2) = annotation('arrow',[0,0],[0,0]); % upper left
ano(3) = annotation('arrow',[0,0],[0,0]); % lower right
ano(4) = annotation('arrow',[0,0],[0,0]); % upper right
set(ano,options.arrows.Names,options.arrows.Values);
% output structure
zoom_utils = struct();
zoom_utils.source = axes2zoom;
zoom_utils.axes = zoomAxes;
zoom_utils.rectangle = rec;
zoom_utils.arrows = ano;
set(zoomAxes,'UserData',zoom_utils);
% set the postZoomAction callback of the new axes
Axzoo = zoom(fig);
Axzoo.ActionPostCallback = @postZoomAction;
    function postZoomAction(obj,evd)
        if isfield(evd.Axes.UserData,'source') && ~isempty(evd.Axes.UserData.source) &&  isa(evd.Axes.UserData.source,'matlab.graphics.axis.Axes') && ishandle(evd.Axes.UserData.source)
            axes2zoom_f = evd.Axes.UserData.source;
            if isfield(evd.Axes.UserData,'rectangle') && ~isempty(evd.Axes.UserData.rectangle) && isa(evd.Axes.UserData.rectangle,'matlab.graphics.primitive.Rectangle')
                rec_fun = evd.Axes.UserData.rectangle;
                newXLim = evd.Axes.XLim;
                newYLim = evd.Axes.YLim;
                set(rec_fun,'Position',[newXLim(1), newYLim(1), diff(newXLim), diff(newYLim)]);
                
                if isfield(evd.Axes.UserData,'arrows') && ~isempty(evd.Axes.UserData.arrows) && isa(evd.Axes.UserData.arrows,'matlab.graphics.shape.Arrow')
                    ano_f = evd.Axes.UserData.arrows;
                    % zoomed in plot:
                    posXLp_f = evd.Axes.Position(1);
                    posXRp_f = sum(evd.Axes.Position([1,3]));
                    posYDp_f = evd.Axes.Position(2);
                    posYUp_f = sum(evd.Axes.Position([2,4]));
                    % zoom rectangle:
                    posXLr_f = axes2zoom_f.Position(1)+axes2zoom_f.Position(3)/diff(axes2zoom_f.XLim)*(rec_fun.Position(1)-axes2zoom_f.XLim(1));
                    posXRr_f = axes2zoom_f.Position(1)+axes2zoom_f.Position(3)/diff(axes2zoom_f.XLim)*(sum(rec_fun.Position([1,3]))-axes2zoom_f.XLim(1));
                    posYDr_f = axes2zoom_f.Position(2)+axes2zoom_f.Position(4)/diff(axes2zoom_f.YLim)*(rec_fun.Position(2)-axes2zoom_f.YLim(1));
                    posYUr_f = axes2zoom_f.Position(2)+axes2zoom_f.Position(4)/diff(axes2zoom_f.YLim)*(sum(rec_fun.Position([2,4]))-axes2zoom_f.YLim(1));
                    set(ano_f(1),'X',[posXLr_f,posXLp_f],'Y',[posYDr_f,posYDp_f]);
                    set(ano_f(2),'X',[posXLr_f,posXLp_f],'Y',[posYUr_f,posYUp_f]);
                    set(ano_f(3),'X',[posXRr_f,posXRp_f],'Y',[posYDr_f,posYDp_f]);
                    set(ano_f(4),'X',[posXRr_f,posXRp_f],'Y',[posYUr_f,posYUp_f]);
                    
                    % display all
                    set(ano_f,'Visible','on');
                    % turn off if they cross the zoom rectangle
                    if posXLr_f<posXLp_f && posYDr_f<posYDp_f
                        set(ano_f(1),'Visible','off');
                    end
                    if posXLr_f<posXLp_f && posYUr_f>posYUp_f
                        set(ano_f(2),'Visible','off');
                    end
                    if posXRr_f>posXRp_f && posYDr_f<posYDp_f
                        set(ano_f(3),'Visible','off');
                    end
                    if posXRr_f>posXRp_f && posYUr_f>posYUp_f
                        set(ano_f(4),'Visible','off');
                    end
                    % turn off if they cross the zoom axes
                    if posXLr_f>posXLp_f && posYDr_f>posYDp_f
                        set(ano_f(1),'Visible','off');
                    end
                    if posXLr_f>posXLp_f && posYUr_f<posYUp_f
                        set(ano_f(2),'Visible','off');
                    end
                    if posXRr_f<posXRp_f && posYDr_f>posYDp_f
                        set(ano_f(3),'Visible','off');
                    end
                    if posXRr_f<posXRp_f && posYUr_f<posYUp_f
                        set(ano_f(4),'Visible','off');
                    end
                end
            end
        end
    end
Axzoo.Enable = 'on';
% set the postPanAction callback of the new axes
Axpan = pan(fig);
Axpan.ActionPostCallback = @postZoomAction;
Axpan.Enable = 'off';
%% enable dragging to move plot
set(zoomAxes,'ButtonDownFcn',@zoomAxesButtonDown);
    function zoomAxesButtonDown(src,event)
        % redraw when clicked
        evd.Axes = src;
        postZoomAction([],evd);
        fig_f = src.Parent;
        fig_f.Units = 'normalized';
        oldCurPos = fig_f.CurrentPoint;
        oldAxPos = src.Position;
        fig_f.WindowButtonMotionFcn = {@moveZoomAxes,src,oldCurPos,oldAxPos};
        fig_f.WindowButtonUpFcn = @(x,y) set(x,'WindowButtonMotionFcn','');
        
    end
    function moveZoomAxes(src,event,ax2move,oldCurPos,oldAxPos)
        
        offset = src.CurrentPoint-oldCurPos;
        ax2move.Position = oldAxPos + [offset,0,0];
        
        evd.Axes = ax2move;
        postZoomAction([],evd);
    end
%% call the postZoomAction once
obj.Axes = zoomAxes;
postZoomAction([],obj);
end
