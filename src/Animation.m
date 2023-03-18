classdef Animation < handle
    properties
        frame_size = [1280 720];            % Frame size in pixels
        should_refresh = true;              % Clear frame after drawing?
        frame_skip = 1;                     % # of steps to skip
        vid_profile = "Uncompressed AVI";   % Video profile
        vid_linux_convert = true;           % Should convert w/ ffmpeg?
        vid_fps = 20;                       % Video FPS

        % Internal properties
        vid_writer;
        fig;
        ax;
        ppu;
        pends = [];
        offsets = [];
        colors = [];
        default_color = [1 1 1]*.7;         

        % Unimplemented
        should_loop = true;
        loop_delay = 5;
    end
    
    methods
        % Add a pendulum simulation to the animation
        % offset - where its origin should be
        % cart_col, link_col - styling of cart and link
        function add_sim_result(obj, res, offset, cart_col, link_col)
            if ~isempty(obj.pends)
                if (length(obj.pends(1).t) ~= length(res.t))
                    error("Simulation t values don't match");
                end
            end

            obj.pends = [obj.pends res];
            obj.offsets = [obj.offsets offset];

            % Colors
            alpha = 1;
            if nargin <= 3
                cart_col = obj.default_color;
            end
            
            if nargin <= 4
                white = ones(1, length(cart_col));
                link_col = cart_col + 1/3*(white - cart_col);
            end

            if length(link_col) == 4
                alpha = link_col(4);
            end

            color = struct('cc', cart_col, 'lc', link_col, 'a', alpha);
            obj.colors = [obj.colors; color];
        end

        % Turn recording on, video will be saved after animation
        function create_video(obj, name)
            obj.vid_writer = VideoWriter(name, obj.vid_profile);
            obj.vid_writer.FrameRate = obj.vid_fps;
        end

        % Save the frame to the specified file
        function save_image(obj, fname)
            img = getframe(obj.fig, obj.ax.OuterPosition);
            imwrite(img.cdata, fname);
        end

        % Internal method - set up figure
        function obj = setup_figure(obj, viewport)
            % Create figure and axis
            obj.fig = figure();
            obj.ax = axes(obj.fig);
            
            obj.fig.Color = 'w';
            obj.fig.GraphicsSmoothing = 'on';
            
            % Set labels and ticks
            set(obj.ax, 'TickLabelInterpreter', 'latex');
            set(obj.ax, 'FontSize', 18);
            xlabel("$x$ (m)", 'interpreter', 'latex');
            ylabel("$y$ (m)", 'interpreter', 'latex');
            
            grid on;
            hold on;
            
            % Set axis and figure size, position, and aspect ratio
            set(obj.ax, 'Units', 'pixels');
            axis(viewport);
            wpos = get(obj.fig, 'Position');
            set(obj.fig, 'Position', [wpos(1:2) obj.frame_size+200]);
            set(obj.ax, 'OuterPosition', [100 100 obj.frame_size]);
            
            % Find points per unit
            obj.ax.Units = 'points';
            ax_pos = tightPosition(obj.ax);
            obj.ppu = ax_pos(3)/diff(xlim(obj.ax));
            obj.ax.Units = 'pixels';
        end

        % Animate the pendulums
        function animate(obj)
            % Set up figure
            if isempty(obj.pends)
                error("Add at least one simulation result");
            end

            if isempty(obj.fig)
                obj.setup_figure(obj.calc_viewport());
            end

            if ~isempty(obj.vid_writer)
                open(obj.vid_writer);
            end
            
            % Draw cart and pendulum
            w = 1;
            h = 0.4;
            axx = axis();

            for i=1:obj.frame_skip:length(obj.pends(1).t)
                % Clear previous frame if applicable
                if obj.should_refresh
                    cla(obj.ax);
                    axis(axx);
                    plot(axx(1:2), [0 0], 'k:', 'LineWidth', 0.05*obj.ppu);
                end

                for p=1:length(obj.pends)
                    % Get pendulum link positions
                    pend = obj.pends(p);
                    qs = [pend.x(i)+obj.offsets(p); pend.th(:, i)];
                    xs = pend.model.get_kinematics(qs);
                    x = repelem(xs, 1, 2);

                    % Plot pendulum
                    rectangle('EdgeColor', [1 1 1 0.25], ...
                        'Position', [xs(1, 1)-w/2, xs(2, 1)-h/2, w, h], ...
                        'FaceColor', obj.colors(p).cc, 'Curvature', 0.5);
                    plot(x(1, 2:end-1), x(2, 2:end-1),  'LineWidth', ...
                        obj.ppu*.1, 'Color', obj.colors(p).lc);
                    
                    scatter(xs(1, :), xs(2, :), obj.ppu*2,  ...
                        obj.colors(p).lc(1:3), 'filled', ...
                        'MarkerFaceAlpha', obj.colors(p).a);
                    scatter(x(1, 2:end-1), x(2, 2:end-1), obj.ppu/2, ...
                        'white', 'filled');
                end

                drawnow; % Commit output to screen
                if ~isempty(obj.vid_writer)
                    % Save frame to recording
                    img = getframe(obj.fig, obj.ax.OuterPosition);
                    writeVideo(obj.vid_writer, img);
                end
            end

            if ~isempty(obj.vid_writer)
                close(obj.vid_writer);

                if (obj.vid_linux_convert)
                    % Convert .avi to .mp4 because MATLAB doesn't have an
                    % H.264 encoder on Linux >:(
                    fname = obj.vid_writer.Filename;
                    fname2 = strrep(fname, 'avi', 'mp4');
                    cmd = "ffmpeg -i %s -vcodec libx264 -acodec aac %s";
                    system(sprintf(cmd, fname, fname2));
                    delete(fname);
                end
            end
        end

        function view_size = calc_viewport(obj)
            % Find tight bounding box of pendulums
            view_size = [Inf -Inf Inf -Inf];

            % Iterate through each pendulum at each timestep
            for p=1:length(obj.pends)
                pend = obj.pends(p);
                for i=1:length(pend.t)
                    % Find its bounding box
                    qs = [pend.x(i)+obj.offsets(p); pend.th(:, i)];
                    xl = pend.model.get_kinematics(qs);
                    ux = max(xl(1, :)); lx = min(xl(1, :));
                    uy = max(xl(2, :)); ly = min(xl(2, :));

                    % Enlarge view_size to include the bounding box
                    view_size(1) = min(view_size(1), lx);
                    view_size(2) = max(view_size(2), ux);
                    view_size(3) = min(view_size(3), ly);
                    view_size(4) = max(view_size(4), uy);
                end
            end

            % Expand viewport to be the correct aspect ratio
            view_size = view_size + [-1 1 -1 1]/4;
            w = diff(view_size(1:2));
            h = diff(view_size(3:4));
            ar = obj.frame_size(1)/obj.frame_size(2);

            if (w > h*ar)
                % Expand height about center
                cy = sum(view_size(3:4))/2;
                fac = w/ar/h;
                view_size(3:4) = fac*(view_size(3:4)-cy)+cy;
            else
                % Expand width about center
                cx = sum(view_size(1:2))/2;
                fac = h*ar/w;
                view_size(1:2) = fac*(view_size(1:2)-cx)+cx;
            end
        end
    end
end

